import React from 'react';
import {
  Box,
  Grid,
  Paper,
  Typography,
  Card,
  CardContent,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  Chip,
  LinearProgress,
} from '@mui/material';
import {
  CheckCircle as CheckCircleIcon,
  Error as ErrorIcon,
  Warning as WarningIcon,
  Schedule as ScheduleIcon,
  Map as MapIcon,
  Image as ImageIcon,
  Assignment as AssignmentIcon,
} from '@mui/icons-material';
import { useAppStore } from '../hooks/useAppStore';

export const Dashboard: React.FC = () => {
  const {
    systemStatus,
    rosConnected,
    costmaps,
    satelliteImages,
    activeJobs,
    missions,
  } = useAppStore();

  const getStatusIcon = (connected: boolean) => {
    return connected ? (
      <CheckCircleIcon color="success" />
    ) : (
      <ErrorIcon color="error" />
    );
  };

  const getJobStatusColor = (status: string) => {
    switch (status) {
      case 'completed':
        return 'success';
      case 'failed':
        return 'error';
      case 'running':
        return 'primary';
      case 'pending':
        return 'warning';
      default:
        return 'default';
    }
  };

  const recentJobs = activeJobs.slice(0, 5);
  const recentMissions = missions.slice(0, 3);

  return (
    <Box sx={{ p: 3 }}>
      <Typography variant="h4" gutterBottom>
        Dashboard
      </Typography>

      {/* System Status Cards */}
      <Grid container spacing={3} sx={{ mb: 4 }}>
        <Grid item xs={12} md={3}>
          <Card>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 2 }}>
                {getStatusIcon(rosConnected)}
                <Typography variant="h6" sx={{ ml: 1 }}>
                  ROS Connection
                </Typography>
              </Box>
              <Typography variant="body2" color="text.secondary">
                {rosConnected ? 'Connected' : 'Disconnected'}
              </Typography>
            </CardContent>
          </Card>
        </Grid>

        <Grid item xs={12} md={3}>
          <Card>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 2 }}>
                <MapIcon color="primary" />
                <Typography variant="h6" sx={{ ml: 1 }}>
                  Costmaps
                </Typography>
              </Box>
              <Typography variant="h4">
                {costmaps.length}
              </Typography>
              <Typography variant="body2" color="text.secondary">
                Total costmaps
              </Typography>
            </CardContent>
          </Card>
        </Grid>

        <Grid item xs={12} md={3}>
          <Card>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 2 }}>
                <ImageIcon color="primary" />
                <Typography variant="h6" sx={{ ml: 1 }}>
                  Images
                </Typography>
              </Box>
              <Typography variant="h4">
                {satelliteImages.length}
              </Typography>
              <Typography variant="body2" color="text.secondary">
                Satellite images
              </Typography>
            </CardContent>
          </Card>
        </Grid>

        <Grid item xs={12} md={3}>
          <Card>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 2 }}>
                <AssignmentIcon color="primary" />
                <Typography variant="h6" sx={{ ml: 1 }}>
                  Active Jobs
                </Typography>
              </Box>
              <Typography variant="h4">
                {activeJobs.length}
              </Typography>
              <Typography variant="body2" color="text.secondary">
                Processing jobs
              </Typography>
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      <Grid container spacing={3}>
        {/* Recent Processing Jobs */}
        <Grid item xs={12} md={6}>
          <Paper sx={{ p: 3 }}>
            <Typography variant="h6" gutterBottom>
              Recent Processing Jobs
            </Typography>
            {recentJobs.length > 0 ? (
              <List>
                {recentJobs.map((job) => (
                  <ListItem key={job.id}>
                    <ListItemIcon>
                      {job.status === 'running' ? (
                        <ScheduleIcon color="primary" />
                      ) : job.status === 'completed' ? (
                        <CheckCircleIcon color="success" />
                      ) : job.status === 'failed' ? (
                        <ErrorIcon color="error" />
                      ) : (
                        <WarningIcon color="warning" />
                      )}
                    </ListItemIcon>
                    <ListItemText
                      primary={
                        <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                          <Typography variant="body1">
                            {job.type.replace('_', ' ').toUpperCase()}
                          </Typography>
                          <Chip
                            size="small"
                            label={job.status}
                            color={getJobStatusColor(job.status) as any}
                          />
                        </Box>
                      }
                      secondary={
                        <Box>
                          <Typography variant="body2" color="text.secondary">
                            Started: {new Date(job.created_at).toLocaleString()}
                          </Typography>
                          {job.status === 'running' && (
                            <LinearProgress
                              variant="determinate"
                              value={job.progress}
                              sx={{ mt: 1 }}
                            />
                          )}
                        </Box>
                      }
                    />
                  </ListItem>
                ))}
              </List>
            ) : (
              <Typography variant="body2" color="text.secondary">
                No recent processing jobs
              </Typography>
            )}
          </Paper>
        </Grid>

        {/* System Services Status */}
        <Grid item xs={12} md={6}>
          <Paper sx={{ p: 3 }}>
            <Typography variant="h6" gutterBottom>
              System Services
            </Typography>
            <List>
              <ListItem>
                <ListItemIcon>
                  {getStatusIcon(systemStatus?.services_available.generate_costmap || false)}
                </ListItemIcon>
                <ListItemText
                  primary="Generate Costmap"
                  secondary={
                    systemStatus?.services_available.generate_costmap
                      ? 'Service available'
                      : 'Service unavailable'
                  }
                />
              </ListItem>
              <ListItem>
                <ListItemIcon>
                  {getStatusIcon(systemStatus?.services_available.update_costmap || false)}
                </ListItemIcon>
                <ListItemText
                  primary="Update Costmap"
                  secondary={
                    systemStatus?.services_available.update_costmap
                      ? 'Service available'
                      : 'Service unavailable'
                  }
                />
              </ListItem>
              <ListItem>
                <ListItemIcon>
                  {getStatusIcon(systemStatus?.services_available.get_costmap_info || false)}
                </ListItemIcon>
                <ListItemText
                  primary="Get Costmap Info"
                  secondary={
                    systemStatus?.services_available.get_costmap_info
                      ? 'Service available'
                      : 'Service unavailable'
                  }
                />
              </ListItem>
            </List>
          </Paper>
        </Grid>

        {/* Recent Missions */}
        {recentMissions.length > 0 && (
          <Grid item xs={12}>
            <Paper sx={{ p: 3 }}>
              <Typography variant="h6" gutterBottom>
                Recent Missions
              </Typography>
              <List>
                {recentMissions.map((mission) => (
                  <ListItem key={mission.id}>
                    <ListItemIcon>
                      <AssignmentIcon color="primary" />
                    </ListItemIcon>
                    <ListItemText
                      primary={
                        <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                          <Typography variant="body1">
                            {mission.name}
                          </Typography>
                          <Chip
                            size="small"
                            label={mission.status}
                            color={
                              mission.status === 'completed'
                                ? 'success'
                                : mission.status === 'active'
                                ? 'primary'
                                : 'default'
                            }
                          />
                        </Box>
                      }
                      secondary={
                        <Box>
                          <Typography variant="body2" color="text.secondary">
                            {mission.description}
                          </Typography>
                          <Typography variant="body2" color="text.secondary">
                            Waypoints: {mission.waypoints.length} | 
                            Created: {new Date(mission.created_at).toLocaleDateString()}
                          </Typography>
                        </Box>
                      }
                    />
                  </ListItem>
                ))}
              </List>
            </Paper>
          </Grid>
        )}
      </Grid>
    </Box>
  );
};
