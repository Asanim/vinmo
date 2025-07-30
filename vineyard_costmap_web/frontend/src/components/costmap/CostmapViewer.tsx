import React, { useEffect, useRef, useState } from 'react';
import {
  MapContainer,
  useMap,
} from 'react-leaflet';
import {
  Box,
  Paper,
  Typography,
  IconButton,
  Slider,
  Switch,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  List,
  ListItem,
  ListItemText,
  ListItemSecondaryAction,
} from '@mui/material';
import {
  ExpandMore as ExpandMoreIcon,
  ZoomIn as ZoomInIcon,
  ZoomOut as ZoomOutIcon,
  CenterFocusStrong as CenterIcon,
} from '@mui/icons-material';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import { CostmapData, CostmapLayer, LayerType } from '../../types';

interface CostmapViewerProps {
  costmap: CostmapData | null;
  layers: CostmapLayer[];
  onLayerChange: (layerType: LayerType, updates: Partial<CostmapLayer>) => void;
}

// Custom hook to control map
const MapController: React.FC<{
  costmap: CostmapData | null;
  onMapReady: (map: L.Map) => void;
}> = ({ costmap, onMapReady }) => {
  const map = useMap();

  useEffect(() => {
    onMapReady(map);
  }, [map, onMapReady]);

  useEffect(() => {
    if (costmap) {
      // Calculate bounds based on costmap origin and dimensions
      const bounds = L.latLngBounds([
        [costmap.origin.y, costmap.origin.x],
        [
          costmap.origin.y + costmap.height * costmap.resolution,
          costmap.origin.x + costmap.width * costmap.resolution,
        ],
      ]);
      map.fitBounds(bounds);
    }
  }, [costmap, map]);

  return null;
};

// Convert costmap data to image data URL
const costmapToImageUrl = (costmap: CostmapData, layer: CostmapLayer): string => {
  const canvas = document.createElement('canvas');
  canvas.width = costmap.width;
  canvas.height = costmap.height;
  const ctx = canvas.getContext('2d');
  
  if (!ctx) return '';
  
  const imageData = ctx.createImageData(costmap.width, costmap.height);
  const data = imageData.data;
  
  // Convert costmap data to RGBA
  for (let i = 0; i < costmap.data.length; i++) {
    const cost = costmap.data[i];
    const pixelIndex = i * 4;
    
    // Color mapping based on layer type and cost value
    let r = 0, g = 0, b = 0, a = 255;
    
    switch (layer.type) {
      case 'obstacles':
        // Red for obstacles
        r = cost > 50 ? 255 : 0;
        g = 0;
        b = 0;
        a = cost > 50 ? Math.floor(255 * layer.opacity) : 0;
        break;
      case 'free_space':
        // Green for free space
        r = 0;
        g = cost < 50 ? 255 : 0;
        b = 0;
        a = cost < 50 ? Math.floor(255 * layer.opacity) : 0;
        break;
      case 'vine_rows':
        // Blue for vine rows
        r = 0;
        g = 0;
        b = cost > 20 && cost < 80 ? 255 : 0;
        a = cost > 20 && cost < 80 ? Math.floor(255 * layer.opacity) : 0;
        break;
      case 'headlands':
        // Yellow for headlands
        r = cost > 10 ? 255 : 0;
        g = cost > 10 ? 255 : 0;
        b = 0;
        a = cost > 10 ? Math.floor(255 * layer.opacity) : 0;
        break;
    }
    
    data[pixelIndex] = r;
    data[pixelIndex + 1] = g;
    data[pixelIndex + 2] = b;
    data[pixelIndex + 3] = a;
  }
  
  ctx.putImageData(imageData, 0, 0);
  return canvas.toDataURL();
};

export const CostmapViewer: React.FC<CostmapViewerProps> = ({
  costmap,
  layers,
  onLayerChange,
}) => {
  const mapRef = useRef<L.Map | null>(null);
  const [overlayLayers, setOverlayLayers] = useState<Record<string, L.ImageOverlay>>({});

  const handleMapReady = (map: L.Map) => {
    mapRef.current = map;
  };

  // Update overlay layers when costmap or layers change
  useEffect(() => {
    if (!costmap || !mapRef.current) return;

    const map = mapRef.current;
    
    // Remove existing overlays
    Object.values(overlayLayers).forEach(overlay => {
      map.removeLayer(overlay);
    });

    // Create new overlays
    const newOverlays: Record<string, L.ImageOverlay> = {};
    
    layers.forEach(layer => {
      if (layer.visible) {
        const imageUrl = costmapToImageUrl(costmap, layer);
        const bounds = L.latLngBounds([
          [costmap.origin.y, costmap.origin.x],
          [
            costmap.origin.y + costmap.height * costmap.resolution,
            costmap.origin.x + costmap.width * costmap.resolution,
          ],
        ]);
        
        const overlay = L.imageOverlay(imageUrl, bounds, {
          opacity: layer.opacity,
        });
        
        overlay.addTo(map);
        newOverlays[layer.type] = overlay;
      }
    });
    
    setOverlayLayers(newOverlays);
  }, [costmap, layers]);

  const handleZoomIn = () => {
    mapRef.current?.zoomIn();
  };

  const handleZoomOut = () => {
    mapRef.current?.zoomOut();
  };

  const handleCenter = () => {
    if (costmap && mapRef.current) {
      const bounds = L.latLngBounds([
        [costmap.origin.y, costmap.origin.x],
        [
          costmap.origin.y + costmap.height * costmap.resolution,
          costmap.origin.x + costmap.width * costmap.resolution,
        ],
      ]);
      mapRef.current.fitBounds(bounds);
    }
  };

  if (!costmap) {
    return (
      <Paper sx={{ p: 3, height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
        <Typography variant="body1" color="text.secondary">
          No costmap selected. Please select a costmap to view.
        </Typography>
      </Paper>
    );
  }

  return (
    <Box sx={{ height: '100%', display: 'flex' }}>
      {/* Map Container */}
      <Box sx={{ flex: 1, position: 'relative' }}>
        <MapContainer
          center={[costmap.origin.y, costmap.origin.x]}
          zoom={15}
          style={{ height: '100%', width: '100%' }}
          crs={L.CRS.Simple}
        >
          <MapController costmap={costmap} onMapReady={handleMapReady} />
        </MapContainer>
        
        {/* Map Controls */}
        <Box
          sx={{
            position: 'absolute',
            top: 10,
            right: 10,
            zIndex: 1000,
            display: 'flex',
            flexDirection: 'column',
            gap: 1,
          }}
        >
          <IconButton
            size="small"
            onClick={handleZoomIn}
            sx={{ bgcolor: 'background.paper', boxShadow: 1 }}
          >
            <ZoomInIcon />
          </IconButton>
          <IconButton
            size="small"
            onClick={handleZoomOut}
            sx={{ bgcolor: 'background.paper', boxShadow: 1 }}
          >
            <ZoomOutIcon />
          </IconButton>
          <IconButton
            size="small"
            onClick={handleCenter}
            sx={{ bgcolor: 'background.paper', boxShadow: 1 }}
          >
            <CenterIcon />
          </IconButton>
        </Box>
      </Box>

      {/* Layer Controls */}
      <Paper sx={{ width: 300, p: 2, overflow: 'auto' }}>
        <Typography variant="h6" gutterBottom>
          Layer Controls
        </Typography>
        
        {layers.map((layer) => (
          <Accordion key={layer.type} defaultExpanded>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1" sx={{ textTransform: 'capitalize' }}>
                {layer.type.replace('_', ' ')}
              </Typography>
            </AccordionSummary>
            <AccordionDetails>
              <List dense>
                <ListItem>
                  <ListItemText primary="Visible" />
                  <ListItemSecondaryAction>
                    <Switch
                      checked={layer.visible}
                      onChange={(e) =>
                        onLayerChange(layer.type, { visible: e.target.checked })
                      }
                    />
                  </ListItemSecondaryAction>
                </ListItem>
                
                <ListItem>
                  <Box sx={{ width: '100%' }}>
                    <Typography variant="body2" gutterBottom>
                      Opacity: {Math.round(layer.opacity * 100)}%
                    </Typography>
                    <Slider
                      value={layer.opacity}
                      min={0}
                      max={1}
                      step={0.1}
                      onChange={(_, value) =>
                        onLayerChange(layer.type, { opacity: value as number })
                      }
                      disabled={!layer.visible}
                    />
                  </Box>
                </ListItem>
              </List>
            </AccordionDetails>
          </Accordion>
        ))}
        
        {/* Costmap Info */}
        <Box sx={{ mt: 2, p: 2, bgcolor: 'background.default', borderRadius: 1 }}>
          <Typography variant="subtitle2" gutterBottom>
            Costmap Information
          </Typography>
          <Typography variant="body2" color="text.secondary">
            Resolution: {costmap.resolution.toFixed(3)} m/cell
          </Typography>
          <Typography variant="body2" color="text.secondary">
            Dimensions: {costmap.width} × {costmap.height}
          </Typography>
          <Typography variant="body2" color="text.secondary">
            Origin: ({costmap.origin.x.toFixed(2)}, {costmap.origin.y.toFixed(2)})
          </Typography>
          <Typography variant="body2" color="text.secondary">
            Updated: {new Date(costmap.timestamp).toLocaleString()}
          </Typography>
        </Box>
      </Paper>
    </Box>
  );
};
