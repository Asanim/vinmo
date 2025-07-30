from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime
from uuid import UUID

# User schemas
class UserCreate(BaseModel):
    username: str
    email: str
    password: str
    role: str = "operator"

class UserResponse(BaseModel):
    id: UUID
    username: str
    email: str
    role: str
    created_at: datetime

    class Config:
        from_attributes = True

# Satellite Image schemas
class SatelliteImageCreate(BaseModel):
    name: str
    bounds_north: float
    bounds_south: float
    bounds_east: float
    bounds_west: float

class SatelliteImageResponse(BaseModel):
    id: UUID
    name: str
    file_path: str
    width: int
    height: int
    bounds_north: float
    bounds_south: float
    bounds_east: float
    bounds_west: float
    upload_timestamp: datetime
    processed: bool

    class Config:
        from_attributes = True

# Costmap schemas
class CostmapResponse(BaseModel):
    id: UUID
    name: str
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_z: float
    data_file_path: str
    timestamp: datetime
    costmap_metadata: Dict[str, Any]

    class Config:
        from_attributes = True

# Processing Job schemas
class ProcessingJobResponse(BaseModel):
    id: UUID
    type: str
    status: str
    progress: float
    created_at: datetime
    updated_at: datetime
    input_data: Dict[str, Any]
    output_data: Dict[str, Any]

    class Config:
        from_attributes = True

# Mission schemas
class MissionCreate(BaseModel):
    name: str
    description: Optional[str] = None
    costmap_id: UUID
    waypoints: List[Dict[str, Any]] = []

class MissionResponse(BaseModel):
    id: UUID
    name: str
    description: Optional[str]
    costmap_id: UUID
    waypoints: List[Dict[str, Any]]
    status: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

# Parameter Preset schemas
class ParameterPresetCreate(BaseModel):
    name: str
    parameters: Dict[str, Any]

# System Status schemas
class SystemStatusResponse(BaseModel):
    backend_status: str
    database_status: str
    ros_bridge_status: str
    active_jobs: int
    uptime: int

# Batch processing schemas
class BatchJobRequest(BaseModel):
    jobs: List[Dict[str, Any]]

class BatchJobResponse(BaseModel):
    job_ids: List[UUID]
    status: str
