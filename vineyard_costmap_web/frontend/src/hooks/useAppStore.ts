import { create } from 'zustand';
import { 
  CostmapData, 
  SatelliteImage, 
  ProcessingJob, 
  SystemStatus, 
  CostmapParameters, 
  Mission, 
  User,
  CostmapLayer 
} from '../types';

interface AppState {
  // Authentication
  user: User | null;
  isAuthenticated: boolean;
  
  // System Status
  systemStatus: SystemStatus | null;
  rosConnected: boolean;
  
  // Costmaps
  costmaps: CostmapData[];
  selectedCostmap: CostmapData | null;
  costmapLayers: Record<string, CostmapLayer[]>;
  
  // Satellite Images
  satelliteImages: SatelliteImage[];
  selectedImage: SatelliteImage | null;
  
  // Processing Jobs
  processingJobs: ProcessingJob[];
  activeJobs: ProcessingJob[];
  
  // Parameters
  currentParameters: CostmapParameters | null;
  parameterPresets: Array<{ name: string; parameters: CostmapParameters }>;
  
  // Missions
  missions: Mission[];
  selectedMission: Mission | null;
  
  // UI State
  sidebarOpen: boolean;
  loading: boolean;
  error: string | null;
  
  // Actions
  setUser: (user: User | null) => void;
  setAuthenticated: (authenticated: boolean) => void;
  setSystemStatus: (status: SystemStatus) => void;
  setRosConnected: (connected: boolean) => void;
  setCostmaps: (costmaps: CostmapData[]) => void;
  setSelectedCostmap: (costmap: CostmapData | null) => void;
  updateCostmapLayers: (costmapId: string, layers: CostmapLayer[]) => void;
  setSatelliteImages: (images: SatelliteImage[]) => void;
  setSelectedImage: (image: SatelliteImage | null) => void;
  addSatelliteImage: (image: SatelliteImage) => void;
  removeSatelliteImage: (imageId: string) => void;
  setProcessingJobs: (jobs: ProcessingJob[]) => void;
  addProcessingJob: (job: ProcessingJob) => void;
  updateProcessingJob: (jobId: string, updates: Partial<ProcessingJob>) => void;
  removeProcessingJob: (jobId: string) => void;
  setCurrentParameters: (parameters: CostmapParameters) => void;
  setParameterPresets: (presets: Array<{ name: string; parameters: CostmapParameters }>) => void;
  addParameterPreset: (preset: { name: string; parameters: CostmapParameters }) => void;
  removeParameterPreset: (name: string) => void;
  setMissions: (missions: Mission[]) => void;
  setSelectedMission: (mission: Mission | null) => void;
  addMission: (mission: Mission) => void;
  updateMission: (missionId: string, updates: Partial<Mission>) => void;
  removeMission: (missionId: string) => void;
  setSidebarOpen: (open: boolean) => void;
  setLoading: (loading: boolean) => void;
  setError: (error: string | null) => void;
  clearError: () => void;
}

export const useAppStore = create<AppState>((set) => ({
  // Initial state
  user: null,
  isAuthenticated: false,
  systemStatus: null,
  rosConnected: false,
  costmaps: [],
  selectedCostmap: null,
  costmapLayers: {},
  satelliteImages: [],
  selectedImage: null,
  processingJobs: [],
  activeJobs: [],
  currentParameters: null,
  parameterPresets: [],
  missions: [],
  selectedMission: null,
  sidebarOpen: true,
  loading: false,
  error: null,

  // Actions
  setUser: (user) => set({ user }),
  setAuthenticated: (isAuthenticated) => set({ isAuthenticated }),
  
  setSystemStatus: (systemStatus) => set({ systemStatus }),
  setRosConnected: (rosConnected) => set({ rosConnected }),
  
  setCostmaps: (costmaps) => set({ costmaps }),
  setSelectedCostmap: (selectedCostmap) => set({ selectedCostmap }),
  updateCostmapLayers: (costmapId, layers) => 
    set((state) => ({
      costmapLayers: {
        ...state.costmapLayers,
        [costmapId]: layers
      }
    })),
  
  setSatelliteImages: (satelliteImages) => set({ satelliteImages }),
  setSelectedImage: (selectedImage) => set({ selectedImage }),
  addSatelliteImage: (image) => 
    set((state) => ({
      satelliteImages: [...state.satelliteImages, image]
    })),
  removeSatelliteImage: (imageId) =>
    set((state) => ({
      satelliteImages: state.satelliteImages.filter(img => img.id !== imageId),
      selectedImage: state.selectedImage?.id === imageId ? null : state.selectedImage
    })),
  
  setProcessingJobs: (processingJobs) => 
    set(() => ({
      processingJobs,
      activeJobs: processingJobs.filter(job => 
        job.status === 'pending' || job.status === 'running'
      )
    })),
  addProcessingJob: (job) =>
    set((state) => {
      const newJobs = [...state.processingJobs, job];
      return {
        processingJobs: newJobs,
        activeJobs: newJobs.filter(j => 
          j.status === 'pending' || j.status === 'running'
        )
      };
    }),
  updateProcessingJob: (jobId, updates) =>
    set((state) => {
      const newJobs = state.processingJobs.map(job =>
        job.id === jobId ? { ...job, ...updates } : job
      );
      return {
        processingJobs: newJobs,
        activeJobs: newJobs.filter(job => 
          job.status === 'pending' || job.status === 'running'
        )
      };
    }),
  removeProcessingJob: (jobId) =>
    set((state) => {
      const newJobs = state.processingJobs.filter(job => job.id !== jobId);
      return {
        processingJobs: newJobs,
        activeJobs: newJobs.filter(job => 
          job.status === 'pending' || job.status === 'running'
        )
      };
    }),

  setCurrentParameters: (currentParameters) => set({ currentParameters }),
  setParameterPresets: (parameterPresets) => set({ parameterPresets }),
  addParameterPreset: (preset) =>
    set((state) => ({
      parameterPresets: [...state.parameterPresets, preset]
    })),
  removeParameterPreset: (name) =>
    set((state) => ({
      parameterPresets: state.parameterPresets.filter(preset => preset.name !== name)
    })),

  setMissions: (missions) => set({ missions }),
  setSelectedMission: (selectedMission) => set({ selectedMission }),
  addMission: (mission) =>
    set((state) => ({
      missions: [...state.missions, mission]
    })),
  updateMission: (missionId, updates) =>
    set((state) => ({
      missions: state.missions.map(mission =>
        mission.id === missionId ? { ...mission, ...updates } : mission
      ),
      selectedMission: state.selectedMission?.id === missionId 
        ? { ...state.selectedMission, ...updates }
        : state.selectedMission
    })),
  removeMission: (missionId) =>
    set((state) => ({
      missions: state.missions.filter(mission => mission.id !== missionId),
      selectedMission: state.selectedMission?.id === missionId ? null : state.selectedMission
    })),

  setSidebarOpen: (sidebarOpen) => set({ sidebarOpen }),
  setLoading: (loading) => set({ loading }),
  setError: (error) => set({ error }),
  clearError: () => set({ error: null }),
}));
