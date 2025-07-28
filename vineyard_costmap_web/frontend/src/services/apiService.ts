import axios from 'axios';
import { SatelliteImage, ProcessingJob, Mission, User, CostmapData, CostmapParameters } from '../types';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

class APIService {
  private axios = axios.create({
    baseURL: API_BASE_URL,
    timeout: 30000,
    headers: {
      'Content-Type': 'application/json',
    },
  });

  constructor() {
    // Add request interceptor for authentication
    this.axios.interceptors.request.use(
      (config) => {
        const token = localStorage.getItem('auth_token');
        if (token) {
          config.headers.Authorization = `Bearer ${token}`;
        }
        return config;
      },
      (error) => Promise.reject(error)
    );

    // Add response interceptor for error handling
    this.axios.interceptors.response.use(
      (response) => response,
      (error) => {
        if (error.response?.status === 401) {
          // Handle authentication error
          localStorage.removeItem('auth_token');
          window.location.href = '/login';
        }
        return Promise.reject(error);
      }
    );
  }

  // Authentication
  async login(username: string, password: string): Promise<{ token: string; user: User }> {
    const response = await this.axios.post('/api/auth/login', {
      username,
      password,
    });
    return response.data;
  }

  async logout(): Promise<void> {
    await this.axios.post('/api/auth/logout');
    localStorage.removeItem('auth_token');
  }

  async getCurrentUser(): Promise<User> {
    const response = await this.axios.get('/api/auth/me');
    return response.data;
  }

  // Satellite Images
  async uploadSatelliteImage(
    file: File,
    bounds: {
      north: number;
      south: number;
      east: number;
      west: number;
    }
  ): Promise<SatelliteImage> {
    const formData = new FormData();
    formData.append('image', file);
    formData.append('bounds', JSON.stringify(bounds));

    const response = await this.axios.post('/api/satellite-images', formData, {
      headers: {
        'Content-Type': 'multipart/form-data',
      },
    });
    return response.data;
  }

  async getSatelliteImages(): Promise<SatelliteImage[]> {
    const response = await this.axios.get('/api/satellite-images');
    return response.data;
  }

  async getSatelliteImage(id: string): Promise<SatelliteImage> {
    const response = await this.axios.get(`/api/satellite-images/${id}`);
    return response.data;
  }

  async deleteSatelliteImage(id: string): Promise<void> {
    await this.axios.delete(`/api/satellite-images/${id}`);
  }

  // Costmaps
  async getCostmaps(): Promise<CostmapData[]> {
    const response = await this.axios.get('/api/costmaps');
    return response.data;
  }

  async getCostmap(id: string): Promise<CostmapData> {
    const response = await this.axios.get(`/api/costmaps/${id}`);
    return response.data;
  }

  async deleteCostmap(id: string): Promise<void> {
    await this.axios.delete(`/api/costmaps/${id}`);
  }

  async exportCostmap(id: string, format: 'yaml' | 'pgm' | 'json'): Promise<Blob> {
    const response = await this.axios.get(`/api/costmaps/${id}/export`, {
      params: { format },
      responseType: 'blob',
    });
    return response.data;
  }

  // Processing Jobs
  async getProcessingJobs(): Promise<ProcessingJob[]> {
    const response = await this.axios.get('/api/processing-jobs');
    return response.data;
  }

  async getProcessingJob(id: string): Promise<ProcessingJob> {
    const response = await this.axios.get(`/api/processing-jobs/${id}`);
    return response.data;
  }

  async cancelProcessingJob(id: string): Promise<void> {
    await this.axios.post(`/api/processing-jobs/${id}/cancel`);
  }

  // Parameters
  async getDefaultParameters(): Promise<CostmapParameters> {
    const response = await this.axios.get('/api/parameters/default');
    return response.data;
  }

  async saveParameterPreset(name: string, parameters: CostmapParameters): Promise<void> {
    await this.axios.post('/api/parameters/presets', {
      name,
      parameters,
    });
  }

  async getParameterPresets(): Promise<Array<{ name: string; parameters: CostmapParameters }>> {
    const response = await this.axios.get('/api/parameters/presets');
    return response.data;
  }

  async deleteParameterPreset(name: string): Promise<void> {
    await this.axios.delete(`/api/parameters/presets/${name}`);
  }

  // Missions
  async createMission(mission: Omit<Mission, 'id' | 'created_at' | 'updated_at'>): Promise<Mission> {
    const response = await this.axios.post('/api/missions', mission);
    return response.data;
  }

  async getMissions(): Promise<Mission[]> {
    const response = await this.axios.get('/api/missions');
    return response.data;
  }

  async getMission(id: string): Promise<Mission> {
    const response = await this.axios.get(`/api/missions/${id}`);
    return response.data;
  }

  async updateMission(id: string, updates: Partial<Mission>): Promise<Mission> {
    const response = await this.axios.patch(`/api/missions/${id}`, updates);
    return response.data;
  }

  async deleteMission(id: string): Promise<void> {
    await this.axios.delete(`/api/missions/${id}`);
  }

  // System Status
  async getSystemStatus(): Promise<{
    backend_status: 'healthy' | 'degraded' | 'unhealthy';
    database_status: 'connected' | 'disconnected';
    ros_bridge_status: 'connected' | 'disconnected';
    active_jobs: number;
    uptime: number;
  }> {
    const response = await this.axios.get('/api/system/status');
    return response.data;
  }

  // Batch Processing
  async submitBatchJob(imageIds: string[], parameters: CostmapParameters): Promise<string> {
    const response = await this.axios.post('/api/batch-processing', {
      image_ids: imageIds,
      parameters,
    });
    return response.data.batch_id;
  }

  async getBatchJobStatus(batchId: string): Promise<{
    total_jobs: number;
    completed_jobs: number;
    failed_jobs: number;
    status: 'running' | 'completed' | 'failed';
  }> {
    const response = await this.axios.get(`/api/batch-processing/${batchId}`);
    return response.data;
  }
}

export const apiService = new APIService();
