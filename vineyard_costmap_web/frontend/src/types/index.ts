export interface CostmapData {
  id: string;
  name: string;
  width: number;
  height: number;
  resolution: number;
  origin: {
    x: number;
    y: number;
    z: number;
  };
  data: number[];
  timestamp: string;
  metadata: {
    source_image?: string;
    processing_time?: number;
    parameters?: CostmapParameters;
  };
}

export interface CostmapParameters {
  // Vineyard row detection parameters
  row_detection: {
    hough_threshold: number;
    min_line_length: number;
    max_line_gap: number;
    angle_tolerance: number;
  };
  
  // Costmap generation parameters
  costmap_generation: {
    inflation_radius: number;
    cost_scaling_factor: number;
    obstacle_cost: number;
    free_space_cost: number;
  };
  
  // Processing parameters
  processing: {
    image_preprocessing: boolean;
    gaussian_blur: number;
    edge_detection_threshold: number;
  };
}

export interface SatelliteImage {
  id: string;
  name: string;
  url: string;
  width: number;
  height: number;
  bounds: {
    north: number;
    south: number;
    east: number;
    west: number;
  };
  upload_timestamp: string;
  processed: boolean;
  costmap_id?: string;
}

export interface ProcessingJob {
  id: string;
  type: 'costmap_generation' | 'row_detection' | 'parameter_update';
  status: 'pending' | 'running' | 'completed' | 'failed';
  progress: number;
  created_at: string;
  updated_at: string;
  input_data: {
    image_id?: string;
    parameters?: CostmapParameters;
  };
  output_data?: {
    costmap_id?: string;
    error_message?: string;
  };
}

export interface SystemStatus {
  ros_connected: boolean;
  services_available: {
    generate_costmap: boolean;
    update_costmap: boolean;
    get_costmap_info: boolean;
  };
  active_jobs: number;
  last_heartbeat: string;
}

export interface User {
  id: string;
  username: string;
  email: string;
  role: 'admin' | 'operator' | 'viewer';
  created_at: string;
}

export interface Mission {
  id: string;
  name: string;
  description: string;
  costmap_id: string;
  waypoints: Waypoint[];
  status: 'draft' | 'active' | 'completed' | 'paused';
  created_by: string;
  created_at: string;
  updated_at: string;
}

export interface Waypoint {
  id: string;
  sequence: number;
  x: number;
  y: number;
  z: number;
  yaw: number;
  type: 'normal' | 'pause' | 'action' | 'checkpoint';
  tolerance: number;
  metadata?: {
    action?: string;
    duration?: number;
  };
}

export type LayerType = 'obstacles' | 'free_space' | 'vine_rows' | 'headlands';

export interface CostmapLayer {
  type: LayerType;
  visible: boolean;
  opacity: number;
  color: string;
  data: number[];
}

export interface DetectionResult {
  vine_rows: Array<{
    start: { x: number; y: number };
    end: { x: number; y: number };
    angle: number;
    confidence: number;
  }>;
  obstacles: Array<{
    x: number;
    y: number;
    radius: number;
    type: string;
  }>;
  headlands: Array<{
    points: Array<{ x: number; y: number }>;
  }>;
  processing_stats: {
    detection_time: number;
    total_rows: number;
    total_obstacles: number;
  };
}
