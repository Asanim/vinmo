import ROSLIB from 'roslib';
import { CostmapParameters, ProcessingJob, SystemStatus, CostmapData } from '../types';

class ROSService {
  private ros: ROSLIB.Ros | null = null;
  private reconnectInterval: NodeJS.Timeout | null = null;
  private heartbeatInterval: NodeJS.Timeout | null = null;
  private connectionListeners: Array<(connected: boolean) => void> = [];

  constructor() {
    this.connect();
  }

  connect() {
    const wsUrl = import.meta.env.VITE_WS_URL || 'ws://localhost:9091';
    
    try {
      this.ros = new ROSLIB.Ros({
        url: wsUrl
      });

      this.ros.on('connection', () => {
        console.log('Connected to ROS bridge');
        this.notifyConnectionListeners(true);
        this.startHeartbeat();
        if (this.reconnectInterval) {
          clearInterval(this.reconnectInterval);
          this.reconnectInterval = null;
        }
      });

      this.ros.on('error', (error: any) => {
        console.error('ROS connection error:', error);
        this.notifyConnectionListeners(false);
      });

      this.ros.on('close', () => {
        console.log('ROS connection closed');
        this.notifyConnectionListeners(false);
        this.stopHeartbeat();
        this.scheduleReconnect();
      });

    } catch (error) {
      console.error('Failed to create ROS connection:', error);
      this.scheduleReconnect();
    }
  }

  private scheduleReconnect() {
    if (!this.reconnectInterval) {
      this.reconnectInterval = setInterval(() => {
        console.log('Attempting to reconnect to ROS...');
        this.connect();
      }, 5000);
    }
  }

  private startHeartbeat() {
    this.heartbeatInterval = setInterval(() => {
      if (this.ros && this.ros.isConnected) {
        // Send heartbeat - can be a simple service check
        this.checkSystemStatus().catch(() => {
          // Heartbeat failed
        });
      }
    }, 30000); // 30 second heartbeat
  }

  private stopHeartbeat() {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
      this.heartbeatInterval = null;
    }
  }

  private notifyConnectionListeners(connected: boolean) {
    this.connectionListeners.forEach(listener => listener(connected));
  }

  onConnectionChange(callback: (connected: boolean) => void) {
    this.connectionListeners.push(callback);
    // Immediately notify with current status
    callback(this.isConnected());
  }

  isConnected(): boolean {
    return this.ros?.isConnected || false;
  }

  // Generate costmap from satellite imagery
  async generateCostmap(
    imageData: string, 
    parameters: CostmapParameters
  ): Promise<{ success: boolean; job_id?: string; error?: string }> {
    return new Promise((resolve, reject) => {
      if (!this.ros || !this.ros.isConnected) {
        reject(new Error('ROS not connected'));
        return;
      }

      const generateCostmapService = new ROSLIB.Service({
        ros: this.ros,
        name: '/costmap/generate_costmap',
        serviceType: 'vineyard_mower_interfaces/GenerateCostmap'
      });

      const request = new ROSLIB.ServiceRequest({
        image_data: imageData,
        parameters: {
          row_detection: parameters.row_detection,
          costmap_generation: parameters.costmap_generation,
          processing: parameters.processing
        }
      });

      generateCostmapService.callService(request, (result: any) => {
        if (result.success) {
          resolve({ 
            success: true, 
            job_id: result.job_id 
          });
        } else {
          resolve({ 
            success: false, 
            error: result.message 
          });
        }
      }, (error: string) => {
        reject(new Error(error));
      });
    });
  }

  // Update specific costmap layer
  async updateCostmapLayer(
    costmapId: string,
    layerName: string,
    layerData: any
  ): Promise<{ success: boolean; error?: string }> {
    return new Promise((resolve, reject) => {
      if (!this.ros || !this.ros.isConnected) {
        reject(new Error('ROS not connected'));
        return;
      }

      const updateLayerService = new ROSLIB.Service({
        ros: this.ros,
        name: '/costmap/update_costmap_layer',
        serviceType: 'vineyard_mower_interfaces/UpdateCostmapLayer'
      });

      const request = new ROSLIB.ServiceRequest({
        costmap_id: costmapId,
        layer_name: layerName,
        layer_data: layerData
      });

      updateLayerService.callService(request, (result: any) => {
        resolve({
          success: result.success,
          error: result.success ? undefined : result.message
        });
      }, (error: string) => {
        reject(new Error(error));
      });
    });
  }

  // Get costmap information and status
  async getCostmapInfo(costmapId: string): Promise<CostmapData> {
    return new Promise((resolve, reject) => {
      if (!this.ros || !this.ros.isConnected) {
        reject(new Error('ROS not connected'));
        return;
      }

      const getCostmapInfoService = new ROSLIB.Service({
        ros: this.ros,
        name: '/costmap/get_costmap_info',
        serviceType: 'vineyard_mower_interfaces/GetCostmapInfo'
      });

      const request = new ROSLIB.ServiceRequest({
        costmap_id: costmapId
      });

      getCostmapInfoService.callService(request, (result: any) => {
        if (result.success) {
          resolve({
            id: result.costmap_id,
            name: result.name || 'Unnamed Costmap',
            width: result.width,
            height: result.height,
            resolution: result.resolution,
            origin: result.origin,
            data: result.data,
            timestamp: result.timestamp,
            metadata: result.metadata || {}
          });
        } else {
          reject(new Error(result.message));
        }
      }, (error: string) => {
        reject(new Error(error));
      });
    });
  }

  // Subscribe to costmap updates
  subscribeToCostmapUpdates(callback: (costmap: CostmapData) => void) {
    if (!this.ros || !this.ros.isConnected) {
      console.warn('Cannot subscribe: ROS not connected');
      return;
    }

    const costmapTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/costmap/updates',
      messageType: 'nav_msgs/OccupancyGrid'
    });

    costmapTopic.subscribe((message: any) => {
      const costmapData: CostmapData = {
        id: message.header.frame_id,
        name: `Costmap_${Date.now()}`,
        width: message.info.width,
        height: message.info.height,
        resolution: message.info.resolution,
        origin: {
          x: message.info.origin.position.x,
          y: message.info.origin.position.y,
          z: message.info.origin.position.z
        },
        data: message.data,
        timestamp: new Date().toISOString(),
        metadata: {}
      };
      callback(costmapData);
    });
  }

  // Subscribe to processing job updates
  subscribeToJobUpdates(callback: (job: ProcessingJob) => void) {
    if (!this.ros || !this.ros.isConnected) {
      console.warn('Cannot subscribe: ROS not connected');
      return;
    }

    const jobTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/costmap/job_updates',
      messageType: 'std_msgs/String'
    });

    jobTopic.subscribe((message: any) => {
      try {
        const job = JSON.parse(message.data) as ProcessingJob;
        callback(job);
      } catch (error) {
        console.error('Failed to parse job update:', error);
      }
    });
  }

  // Check system status
  async checkSystemStatus(): Promise<SystemStatus> {
    return new Promise((resolve) => {
      const status: SystemStatus = {
        ros_connected: this.isConnected(),
        services_available: {
          generate_costmap: false,
          update_costmap: false,
          get_costmap_info: false
        },
        active_jobs: 0,
        last_heartbeat: new Date().toISOString()
      };

      if (!this.ros || !this.ros.isConnected) {
        resolve(status);
        return;
      }

      // Check if services are available
      const serviceNames = [
        '/costmap/generate_costmap',
        '/costmap/update_costmap_layer',
        '/costmap/get_costmap_info'
      ];

      let completedChecks = 0;
      
      serviceNames.forEach((serviceName) => {
        const service = new ROSLIB.Service({
          ros: this.ros!,
          name: serviceName,
          serviceType: 'std_srvs/Empty'
        });

        // Simple ping to check if service exists
        const request = new ROSLIB.ServiceRequest({});
        
        service.callService(request, () => {
          // Service available
          const serviceKey = serviceName.split('/').pop()?.replace('_costmap', '') as keyof typeof status.services_available;
          if (serviceKey) {
            status.services_available[serviceKey as keyof typeof status.services_available] = true;
          }
          completedChecks++;
          if (completedChecks === serviceNames.length) {
            resolve(status);
          }
        }, () => {
          // Service not available or error
          completedChecks++;
          if (completedChecks === serviceNames.length) {
            resolve(status);
          }
        });
      });
    });
  }

  disconnect() {
    if (this.reconnectInterval) {
      clearInterval(this.reconnectInterval);
      this.reconnectInterval = null;
    }
    this.stopHeartbeat();
    
    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }
  }
}

export const rosService = new ROSService();
