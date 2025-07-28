import React, { useState, useEffect } from 'react';
import {
  Box,
  Paper,
  Typography,
  TextField,
  Slider,
  Switch,
  FormControlLabel,
  Button,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Grid,
  Alert,
  Snackbar,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  List,
  ListItem,
  ListItemText,
  ListItemSecondaryAction,
  IconButton,
} from '@mui/material';
import {
  ExpandMore as ExpandMoreIcon,
  Save as SaveIcon,
  Refresh as RefreshIcon,
  Delete as DeleteIcon,
} from '@mui/icons-material';
import { CostmapParameters } from '../../types';
import { useAppStore } from '../../hooks/useAppStore';
import { apiService } from '../../services/apiService';

interface ParameterControlPanelProps {
  parameters: CostmapParameters;
  onChange: (parameters: CostmapParameters) => void;
  onSave?: (parameters: CostmapParameters) => void;
}

export const ParameterControlPanel: React.FC<ParameterControlPanelProps> = ({
  parameters,
  onChange,
  onSave,
}) => {
  const { parameterPresets, setParameterPresets, setError } = useAppStore();
  const [presetDialogOpen, setPresetDialogOpen] = useState(false);
  const [presetName, setPresetName] = useState('');
  const [snackbarOpen, setSnackbarOpen] = useState(false);
  const [snackbarMessage, setSnackbarMessage] = useState('');

  useEffect(() => {
    loadPresets();
  }, []);

  const loadPresets = async () => {
    try {
      const presets = await apiService.getParameterPresets();
      setParameterPresets(presets);
    } catch (error) {
      setError(`Failed to load parameter presets: ${error}`);
    }
  };

  const handleParameterChange = (
    section: keyof CostmapParameters,
    key: string,
    value: number | boolean
  ) => {
    const updated = {
      ...parameters,
      [section]: {
        ...parameters[section],
        [key]: value,
      },
    };
    onChange(updated);
  };

  const handleSavePreset = async () => {
    if (!presetName.trim()) return;

    try {
      await apiService.saveParameterPreset(presetName, parameters);
      await loadPresets();
      setPresetDialogOpen(false);
      setPresetName('');
      setSnackbarMessage('Parameter preset saved successfully');
      setSnackbarOpen(true);
    } catch (error) {
      setError(`Failed to save parameter preset: ${error}`);
    }
  };

  const handleLoadPreset = (preset: { name: string; parameters: CostmapParameters }) => {
    onChange(preset.parameters);
    setSnackbarMessage(`Loaded preset: ${preset.name}`);
    setSnackbarOpen(true);
  };

  const handleDeletePreset = async (name: string) => {
    try {
      await apiService.deleteParameterPreset(name);
      await loadPresets();
      setSnackbarMessage(`Deleted preset: ${name}`);
      setSnackbarOpen(true);
    } catch (error) {
      setError(`Failed to delete parameter preset: ${error}`);
    }
  };

  const handleSave = () => {
    if (onSave) {
      onSave(parameters);
      setSnackbarMessage('Parameters saved successfully');
      setSnackbarOpen(true);
    }
  };

  const handleReset = async () => {
    try {
      const defaultParams = await apiService.getDefaultParameters();
      onChange(defaultParams);
      setSnackbarMessage('Parameters reset to defaults');
      setSnackbarOpen(true);
    } catch (error) {
      setError(`Failed to load default parameters: ${error}`);
    }
  };

  return (
    <Box sx={{ p: 2 }}>
      <Paper sx={{ p: 3 }}>
        <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
          <Typography variant="h5">Parameter Control</Typography>
          <Box sx={{ display: 'flex', gap: 1 }}>
            <Button
              variant="outlined"
              startIcon={<RefreshIcon />}
              onClick={handleReset}
            >
              Reset to Defaults
            </Button>
            <Button
              variant="outlined"
              onClick={() => setPresetDialogOpen(true)}
            >
              Save as Preset
            </Button>
            <Button
              variant="contained"
              startIcon={<SaveIcon />}
              onClick={handleSave}
            >
              Apply Parameters
            </Button>
          </Box>
        </Box>

        {/* Row Detection Parameters */}
        <Accordion defaultExpanded>
          <AccordionSummary expandIcon={<ExpandMoreIcon />}>
            <Typography variant="h6">Row Detection Parameters</Typography>
          </AccordionSummary>
          <AccordionDetails>
            <Grid container spacing={3}>
              <Grid item xs={12} md={6}>
                <Typography gutterBottom>
                  Hough Threshold: {parameters.row_detection.hough_threshold}
                </Typography>
                <Slider
                  value={parameters.row_detection.hough_threshold}
                  min={50}
                  max={300}
                  step={10}
                  onChange={(_, value) =>
                    handleParameterChange('row_detection', 'hough_threshold', value as number)
                  }
                />
              </Grid>
              <Grid item xs={12} md={6}>
                <Typography gutterBottom>
                  Min Line Length: {parameters.row_detection.min_line_length}
                </Typography>
                <Slider
                  value={parameters.row_detection.min_line_length}
                  min={10}
                  max={200}
                  step={5}
                  onChange={(_, value) =>
                    handleParameterChange('row_detection', 'min_line_length', value as number)
                  }
                />
              </Grid>
              <Grid item xs={12} md={6}>
                <Typography gutterBottom>
                  Max Line Gap: {parameters.row_detection.max_line_gap}
                </Typography>
                <Slider
                  value={parameters.row_detection.max_line_gap}
                  min={5}
                  max={50}
                  step={1}
                  onChange={(_, value) =>
                    handleParameterChange('row_detection', 'max_line_gap', value as number)
                  }
                />
              </Grid>
              <Grid item xs={12} md={6}>
                <Typography gutterBottom>
                  Angle Tolerance: {parameters.row_detection.angle_tolerance}°
                </Typography>
                <Slider
                  value={parameters.row_detection.angle_tolerance}
                  min={1}
                  max={30}
                  step={1}
                  onChange={(_, value) =>
                    handleParameterChange('row_detection', 'angle_tolerance', value as number)
                  }
                />
              </Grid>
            </Grid>
          </AccordionDetails>
        </Accordion>

        {/* Costmap Generation Parameters */}
        <Accordion defaultExpanded>
          <AccordionSummary expandIcon={<ExpandMoreIcon />}>
            <Typography variant="h6">Costmap Generation Parameters</Typography>
          </AccordionSummary>
          <AccordionDetails>
            <Grid container spacing={3}>
              <Grid item xs={12} md={6}>
                <Typography gutterBottom>
                  Inflation Radius: {parameters.costmap_generation.inflation_radius}m
                </Typography>
                <Slider
                  value={parameters.costmap_generation.inflation_radius}
                  min={0.1}
                  max={2.0}
                  step={0.1}
                  onChange={(_, value) =>
                    handleParameterChange('costmap_generation', 'inflation_radius', value as number)
                  }
                />
              </Grid>
              <Grid item xs={12} md={6}>
                <Typography gutterBottom>
                  Cost Scaling Factor: {parameters.costmap_generation.cost_scaling_factor}
                </Typography>
                <Slider
                  value={parameters.costmap_generation.cost_scaling_factor}
                  min={1}
                  max={20}
                  step={1}
                  onChange={(_, value) =>
                    handleParameterChange('costmap_generation', 'cost_scaling_factor', value as number)
                  }
                />
              </Grid>
              <Grid item xs={12} md={6}>
                <TextField
                  fullWidth
                  label="Obstacle Cost"
                  type="number"
                  value={parameters.costmap_generation.obstacle_cost}
                  onChange={(e) =>
                    handleParameterChange('costmap_generation', 'obstacle_cost', parseInt(e.target.value))
                  }
                  inputProps={{ min: 0, max: 255 }}
                />
              </Grid>
              <Grid item xs={12} md={6}>
                <TextField
                  fullWidth
                  label="Free Space Cost"
                  type="number"
                  value={parameters.costmap_generation.free_space_cost}
                  onChange={(e) =>
                    handleParameterChange('costmap_generation', 'free_space_cost', parseInt(e.target.value))
                  }
                  inputProps={{ min: 0, max: 255 }}
                />
              </Grid>
            </Grid>
          </AccordionDetails>
        </Accordion>

        {/* Processing Parameters */}
        <Accordion defaultExpanded>
          <AccordionSummary expandIcon={<ExpandMoreIcon />}>
            <Typography variant="h6">Processing Parameters</Typography>
          </AccordionSummary>
          <AccordionDetails>
            <Grid container spacing={3}>
              <Grid item xs={12}>
                <FormControlLabel
                  control={
                    <Switch
                      checked={parameters.processing.image_preprocessing}
                      onChange={(e) =>
                        handleParameterChange('processing', 'image_preprocessing', e.target.checked)
                      }
                    />
                  }
                  label="Enable Image Preprocessing"
                />
              </Grid>
              <Grid item xs={12} md={6}>
                <Typography gutterBottom>
                  Gaussian Blur: {parameters.processing.gaussian_blur}
                </Typography>
                <Slider
                  value={parameters.processing.gaussian_blur}
                  min={0}
                  max={10}
                  step={1}
                  onChange={(_, value) =>
                    handleParameterChange('processing', 'gaussian_blur', value as number)
                  }
                  disabled={!parameters.processing.image_preprocessing}
                />
              </Grid>
              <Grid item xs={12} md={6}>
                <Typography gutterBottom>
                  Edge Detection Threshold: {parameters.processing.edge_detection_threshold}
                </Typography>
                <Slider
                  value={parameters.processing.edge_detection_threshold}
                  min={50}
                  max={300}
                  step={10}
                  onChange={(_, value) =>
                    handleParameterChange('processing', 'edge_detection_threshold', value as number)
                  }
                  disabled={!parameters.processing.image_preprocessing}
                />
              </Grid>
            </Grid>
          </AccordionDetails>
        </Accordion>

        {/* Parameter Presets */}
        {parameterPresets.length > 0 && (
          <Accordion>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="h6">Saved Presets</Typography>
            </AccordionSummary>
            <AccordionDetails>
              <List>
                {parameterPresets.map((preset) => (
                  <ListItem key={preset.name}>
                    <ListItemText
                      primary={preset.name}
                      secondary="Click to load preset"
                      onClick={() => handleLoadPreset(preset)}
                      sx={{ cursor: 'pointer' }}
                    />
                    <ListItemSecondaryAction>
                      <IconButton
                        edge="end"
                        onClick={() => handleDeletePreset(preset.name)}
                      >
                        <DeleteIcon />
                      </IconButton>
                    </ListItemSecondaryAction>
                  </ListItem>
                ))}
              </List>
            </AccordionDetails>
          </Accordion>
        )}
      </Paper>

      {/* Save Preset Dialog */}
      <Dialog open={presetDialogOpen} onClose={() => setPresetDialogOpen(false)}>
        <DialogTitle>Save Parameter Preset</DialogTitle>
        <DialogContent>
          <TextField
            fullWidth
            label="Preset Name"
            value={presetName}
            onChange={(e) => setPresetName(e.target.value)}
            margin="normal"
            placeholder="Enter a name for this preset..."
          />
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setPresetDialogOpen(false)}>Cancel</Button>
          <Button onClick={handleSavePreset} variant="contained">
            Save Preset
          </Button>
        </DialogActions>
      </Dialog>

      {/* Snackbar for notifications */}
      <Snackbar
        open={snackbarOpen}
        autoHideDuration={3000}
        onClose={() => setSnackbarOpen(false)}
      >
        <Alert onClose={() => setSnackbarOpen(false)} severity="success">
          {snackbarMessage}
        </Alert>
      </Snackbar>
    </Box>
  );
};
