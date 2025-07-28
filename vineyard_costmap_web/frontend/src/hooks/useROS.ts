import { useEffect, useCallback } from 'react';
import { rosService } from '../services/rosService';
import { apiService } from '../services/apiService';
import { useAppStore } from './useAppStore';

export const useROS = () => {
  const {
    setRosConnected,
    setSystemStatus,
    addProcessingJob,
    updateProcessingJob,
    setError
  } = useAppStore();

  const checkSystemStatus = useCallback(async () => {
    try {
      const status = await rosService.checkSystemStatus();
      setSystemStatus(status);
      setRosConnected(status.ros_connected);
    } catch (error) {
      console.error('Failed to check system status:', error);
      setRosConnected(false);
    }
  }, [setSystemStatus, setRosConnected]);

  useEffect(() => {
    // Monitor ROS connection status
    rosService.onConnectionChange((connected) => {
      setRosConnected(connected);
      if (connected) {
        checkSystemStatus();
      }
    });

    // Subscribe to processing job updates
    rosService.subscribeToJobUpdates((job) => {
      updateProcessingJob(job.id, job);
    });

    // Initial status check
    checkSystemStatus();

    // Set up periodic status checks
    const statusInterval = setInterval(checkSystemStatus, 30000);

    return () => {
      clearInterval(statusInterval);
    };
  }, [checkSystemStatus, setRosConnected, updateProcessingJob]);

  const generateCostmap = useCallback(async (
    imageData: string,
    parameters: any
  ) => {
    try {
      const result = await rosService.generateCostmap(imageData, parameters);
      if (result.success && result.job_id) {
        // Create a new processing job
        addProcessingJob({
          id: result.job_id,
          type: 'costmap_generation',
          status: 'pending',
          progress: 0,
          created_at: new Date().toISOString(),
          updated_at: new Date().toISOString(),
          input_data: { parameters }
        });
      }
      return result;
    } catch (error) {
      setError(`Failed to generate costmap: ${error}`);
      throw error;
    }
  }, [addProcessingJob, setError]);

  const updateCostmapLayer = useCallback(async (
    costmapId: string,
    layerName: string,
    layerData: any
  ) => {
    try {
      return await rosService.updateCostmapLayer(costmapId, layerName, layerData);
    } catch (error) {
      setError(`Failed to update costmap layer: ${error}`);
      throw error;
    }
  }, [setError]);

  const getCostmapInfo = useCallback(async (costmapId: string) => {
    try {
      return await rosService.getCostmapInfo(costmapId);
    } catch (error) {
      setError(`Failed to get costmap info: ${error}`);
      throw error;
    }
  }, [setError]);

  return {
    generateCostmap,
    updateCostmapLayer,
    getCostmapInfo,
    checkSystemStatus
  };
};
