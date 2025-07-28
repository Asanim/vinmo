from fastapi import FastAPI, Depends, HTTPException, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from sqlalchemy.orm import Session
import uvicorn
import os
from datetime import datetime
import logging

from database import get_db, engine, Base
from models import User, SatelliteImage, CostmapData, ProcessingJob, Mission, ParameterPreset
from schemas import (
    UserCreate, UserResponse, SatelliteImageCreate, SatelliteImageResponse,
    CostmapResponse, ProcessingJobResponse, MissionCreate, MissionResponse,
    ParameterPresetCreate, SystemStatusResponse, BatchJobRequest, BatchJobResponse
)
from auth import get_current_user, create_access_token, verify_password, get_password_hash
from image_processing import process_satellite_image
from ros_integration import ROSIntegration

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create tables
Base.metadata.create_all(bind=engine)

app = FastAPI(
    title="Vineyard Costmap API",
    description="API for managing vineyard costmap generation and satellite imagery processing",
    version="1.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://frontend:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize ROS integration
ros_integration = ROSIntegration()

@app.on_event("startup")
async def startup_event():
    """Initialize services on startup"""
    logger.info("Starting up Vineyard Costmap API")
    await ros_integration.connect()

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown"""
    logger.info("Shutting down Vineyard Costmap API")
    await ros_integration.disconnect()

# Authentication endpoints
@app.post("/api/auth/login")
async def login(username: str, password: str, db: Session = Depends(get_db)):
    """Login endpoint"""
    user = db.query(User).filter(User.username == username).first()
    if not user or not verify_password(password, user.hashed_password):
        raise HTTPException(status_code=401, detail="Invalid credentials")
    
    access_token = create_access_token(data={"sub": user.username})
    return {"token": access_token, "user": UserResponse.from_orm(user)}

@app.get("/api/auth/me", response_model=UserResponse)
async def get_current_user_info(current_user: User = Depends(get_current_user)):
    """Get current user information"""
    return UserResponse.from_orm(current_user)

# Satellite Images endpoints
@app.post("/api/satellite-images", response_model=SatelliteImageResponse)
async def upload_satellite_image(
    file: UploadFile = File(...),
    bounds: str = None,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """Upload a satellite image"""
    import json
    
    if not file.content_type.startswith('image/'):
        raise HTTPException(status_code=400, detail="File must be an image")
    
    # Save file
    upload_dir = "uploads/satellite_images"
    os.makedirs(upload_dir, exist_ok=True)
    file_path = os.path.join(upload_dir, f"{datetime.now().isoformat()}_{file.filename}")
    
    with open(file_path, "wb") as buffer:
        content = await file.read()
        buffer.write(content)
    
    # Parse bounds
    bounds_data = json.loads(bounds) if bounds else {"north": 0, "south": 0, "east": 0, "west": 0}
    
    # Process image to get dimensions
    from PIL import Image
    with Image.open(file_path) as img:
        width, height = img.size
    
    # Create database record
    satellite_image = SatelliteImage(
        name=file.filename,
        file_path=file_path,
        width=width,
        height=height,
        bounds_north=bounds_data["north"],
        bounds_south=bounds_data["south"],
        bounds_east=bounds_data["east"],
        bounds_west=bounds_data["west"],
        upload_timestamp=datetime.utcnow(),
        processed=False
    )
    
    db.add(satellite_image)
    db.commit()
    db.refresh(satellite_image)
    
    return SatelliteImageResponse.from_orm(satellite_image)

@app.get("/api/satellite-images", response_model=list[SatelliteImageResponse])
async def get_satellite_images(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """Get all satellite images"""
    images = db.query(SatelliteImage).all()
    return [SatelliteImageResponse.from_orm(img) for img in images]

@app.delete("/api/satellite-images/{image_id}")
async def delete_satellite_image(
    image_id: str,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """Delete a satellite image"""
    image = db.query(SatelliteImage).filter(SatelliteImage.id == image_id).first()
    if not image:
        raise HTTPException(status_code=404, detail="Image not found")
    
    # Delete file
    if os.path.exists(image.file_path):
        os.remove(image.file_path)
    
    db.delete(image)
    db.commit()
    
    return {"message": "Image deleted successfully"}

# Costmaps endpoints
@app.get("/api/costmaps", response_model=list[CostmapResponse])
async def get_costmaps(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """Get all costmaps"""
    costmaps = db.query(CostmapData).all()
    return [CostmapResponse.from_orm(costmap) for costmap in costmaps]

@app.get("/api/costmaps/{costmap_id}", response_model=CostmapResponse)
async def get_costmap(
    costmap_id: str,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """Get a specific costmap"""
    costmap = db.query(CostmapData).filter(CostmapData.id == costmap_id).first()
    if not costmap:
        raise HTTPException(status_code=404, detail="Costmap not found")
    
    return CostmapResponse.from_orm(costmap)

# Processing Jobs endpoints
@app.get("/api/processing-jobs", response_model=list[ProcessingJobResponse])
async def get_processing_jobs(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """Get all processing jobs"""
    jobs = db.query(ProcessingJob).all()
    return [ProcessingJobResponse.from_orm(job) for job in jobs]

# System Status endpoint
@app.get("/api/system/status", response_model=SystemStatusResponse)
async def get_system_status():
    """Get system status"""
    ros_status = await ros_integration.get_status()
    
    return SystemStatusResponse(
        backend_status="healthy",
        database_status="connected",
        ros_bridge_status="connected" if ros_status["connected"] else "disconnected",
        active_jobs=0,  # TODO: Get from database
        uptime=0  # TODO: Calculate uptime
    )

# Parameter Presets endpoints
@app.get("/api/parameters/presets")
async def get_parameter_presets(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """Get parameter presets"""
    presets = db.query(ParameterPreset).filter(ParameterPreset.user_id == current_user.id).all()
    return [{"name": preset.name, "parameters": preset.parameters} for preset in presets]

@app.post("/api/parameters/presets")
async def save_parameter_preset(
    preset: ParameterPresetCreate,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """Save parameter preset"""
    db_preset = ParameterPreset(
        name=preset.name,
        parameters=preset.parameters,
        user_id=current_user.id,
        created_at=datetime.utcnow()
    )
    db.add(db_preset)
    db.commit()
    
    return {"message": "Preset saved successfully"}

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
