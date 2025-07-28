from sqlalchemy import create_engine, Column, String, Integer, DateTime, Boolean, Float, Text, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.dialects.postgresql import UUID
import uuid
import os

# Database configuration
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://vineyard_user:vineyard_pass@localhost:5432/vineyard_costmap")

engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

def get_db():
    """Get database session"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
