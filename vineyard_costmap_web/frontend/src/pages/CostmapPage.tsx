import React, { useState } from 'react';
import {
  Box,
  Typography,
  Grid,
  Paper,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  SelectChangeEvent,
} from '@mui/material';
import { CostmapViewer } from '../components/costmap/CostmapViewer';
import { useAppStore } from '../hooks/useAppStore';
import { CostmapLayer, LayerType } from '../types';

export const CostmapPage: React.FC = () => {
  const { costmaps, selectedCostmap, setSelectedCostmap, updateCostmapLayers } = useAppStore();
  
  const [layers, setLayers] = useState<CostmapLayer[]>([
    { type: 'obstacles', visible: true, opacity: 0.8, color: '#ff0000', data: [] },
    { type: 'free_space', visible: true, opacity: 0.6, color: '#00ff00', data: [] },
    { type: 'vine_rows', visible: true, opacity: 0.7, color: '#0000ff', data: [] },
    { type: 'headlands', visible: true, opacity: 0.5, color: '#ffff00', data: [] },
  ]);

  const handleCostmapSelect = (event: SelectChangeEvent<string>) => {
    const costmapId = event.target.value;
    const costmap = costmaps.find(c => c.id === costmapId) || null;
    setSelectedCostmap(costmap);
    
    if (costmap) {
      // Layers will be updated when costmap data is loaded
    }
  };

  const handleLayerChange = (layerType: LayerType, updates: Partial<CostmapLayer>) => {
    const updatedLayers = layers.map(layer =>
      layer.type === layerType ? { ...layer, ...updates } : layer
    );
    setLayers(updatedLayers);
    
    if (selectedCostmap) {
      updateCostmapLayers(selectedCostmap.id, updatedLayers);
    }
  };

  return (
    <Box sx={{ p: 3, height: 'calc(100vh - 120px)' }}>
      <Box sx={{ mb: 3 }}>
        <Typography variant="h4" gutterBottom>
          Costmap Viewer
        </Typography>
        
        <Paper sx={{ p: 2 }}>
          <Grid container spacing={2} alignItems="center">
            <Grid item xs={12} md={4}>
              <FormControl fullWidth>
                <InputLabel>Select Costmap</InputLabel>
                <Select
                  value={selectedCostmap?.id || ''}
                  onChange={handleCostmapSelect}
                  label="Select Costmap"
                >
                  {costmaps.map((costmap) => (
                    <MenuItem key={costmap.id} value={costmap.id}>
                      {costmap.name} ({new Date(costmap.timestamp).toLocaleDateString()})
                    </MenuItem>
                  ))}
                </Select>
              </FormControl>
            </Grid>
          </Grid>
        </Paper>
      </Box>

      <Paper sx={{ height: 'calc(100% - 140px)' }}>
        <CostmapViewer
          costmap={selectedCostmap}
          layers={layers}
          onLayerChange={handleLayerChange}
        />
      </Paper>
    </Box>
  );
};
