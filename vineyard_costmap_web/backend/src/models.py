from sqlalchemy import Column, String, Integer, DateTime, Boolean, Float, Text, JSON, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from database import Base
import uuid
from datetime import datetime

class User(Base):
    __tablename__ = "users"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    username = Column(String(50), unique=True, nullable=False)
    email = Column(String(255), unique=True, nullable=False)
    hashed_password = Column(String(255), nullable=False)
    role = Column(String(20), default="operator")
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    satellite_images = relationship("SatelliteImage", back_populates="user")
    missions = relationship("Mission", back_populates="user")
    parameter_presets = relationship("ParameterPreset", back_populates="user")

class SatelliteImage(Base):
    __tablename__ = "satellite_images"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String(255), nullable=False)
    file_path = Column(String(500), nullable=False)
    width = Column(Integer, nullable=False)
    height = Column(Integer, nullable=False)
    bounds_north = Column(Float, nullable=False)
    bounds_south = Column(Float, nullable=False)
    bounds_east = Column(Float, nullable=False)
    bounds_west = Column(Float, nullable=False)
    upload_timestamp = Column(DateTime, default=datetime.utcnow)
    processed = Column(Boolean, default=False)
    costmap_id = Column(UUID(as_uuid=True), ForeignKey("costmaps.id"), nullable=True)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False)
    
    # Relationships
    user = relationship("User", back_populates="satellite_images")
    costmap = relationship("CostmapData", back_populates="satellite_image")

class CostmapData(Base):
    __tablename__ = "costmaps"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String(255), nullable=False)
    width = Column(Integer, nullable=False)
    height = Column(Integer, nullable=False)
    resolution = Column(Float, nullable=False)
    origin_x = Column(Float, nullable=False)
    origin_y = Column(Float, nullable=False)
    origin_z = Column(Float, default=0.0)
    data_file_path = Column(String(500), nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow)
    metadata = Column(JSON, default=dict)
    
    # Relationships
    satellite_image = relationship("SatelliteImage", back_populates="costmap")
    missions = relationship("Mission", back_populates="costmap")

class ProcessingJob(Base):
    __tablename__ = "processing_jobs"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    type = Column(String(50), nullable=False)  # costmap_generation, row_detection, etc.
    status = Column(String(20), default="pending")  # pending, running, completed, failed
    progress = Column(Float, default=0.0)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow)
    input_data = Column(JSON, default=dict)
    output_data = Column(JSON, default=dict)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False)

class Mission(Base):
    __tablename__ = "missions"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String(255), nullable=False)
    description = Column(Text)
    costmap_id = Column(UUID(as_uuid=True), ForeignKey("costmaps.id"), nullable=False)
    waypoints = Column(JSON, default=list)
    status = Column(String(20), default="draft")  # draft, active, completed, paused
    created_by = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    user = relationship("User", back_populates="missions")
    costmap = relationship("CostmapData", back_populates="missions")

class ParameterPreset(Base):
    __tablename__ = "parameter_presets"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String(255), nullable=False)
    parameters = Column(JSON, nullable=False)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    user = relationship("User", back_populates="parameter_presets")
