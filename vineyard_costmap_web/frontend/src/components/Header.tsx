import React from 'react';
import {
  AppBar,
  Toolbar,
  Typography,
  IconButton,
  Badge,
  Box,
  Chip,
  Tooltip,
} from '@mui/material';
import {
  Menu as MenuIcon,
  Notifications as NotificationsIcon,
  AccountCircle as AccountCircleIcon,
  WifiOff as WifiOffIcon,
  Wifi as WifiIcon,
} from '@mui/icons-material';
import { useAppStore } from '../hooks/useAppStore';

interface HeaderProps {
  onMenuClick: () => void;
}

export const Header: React.FC<HeaderProps> = ({ onMenuClick }) => {
  const { 
    user, 
    rosConnected, 
    activeJobs, 
    systemStatus 
  } = useAppStore();

  const getConnectionStatus = () => {
    if (!rosConnected) {
      return { color: 'error' as const, label: 'ROS Disconnected', icon: <WifiOffIcon /> };
    }
    
    if (systemStatus?.services_available) {
      const servicesCount = Object.values(systemStatus.services_available).filter(Boolean).length;
      if (servicesCount === 3) {
        return { color: 'success' as const, label: 'All Services Online', icon: <WifiIcon /> };
      } else if (servicesCount > 0) {
        return { color: 'warning' as const, label: 'Partial Services', icon: <WifiIcon /> };
      }
    }
    
    return { color: 'error' as const, label: 'Services Offline', icon: <WifiOffIcon /> };
  };

  const connectionStatus = getConnectionStatus();

  return (
    <AppBar position="static" elevation={1}>
      <Toolbar>
        <IconButton
          edge="start"
          color="inherit"
          aria-label="menu"
          onClick={onMenuClick}
          sx={{ mr: 2 }}
        >
          <MenuIcon />
        </IconButton>

        <Typography variant="h6" component="div" sx={{ flexGrow: 1 }}>
          Vineyard Costmap Manager
        </Typography>

        <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
          {/* Connection Status */}
          <Tooltip title={connectionStatus.label}>
            <Chip
              icon={connectionStatus.icon}
              label={rosConnected ? 'Connected' : 'Disconnected'}
              color={connectionStatus.color}
              size="small"
              variant="outlined"
            />
          </Tooltip>

          {/* Active Jobs Badge */}
          {activeJobs.length > 0 && (
            <Tooltip title={`${activeJobs.length} active jobs`}>
              <Badge badgeContent={activeJobs.length} color="secondary">
                <IconButton color="inherit">
                  <NotificationsIcon />
                </IconButton>
              </Badge>
            </Tooltip>
          )}

          {/* User Info */}
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <Typography variant="body2">
              {user?.username || 'Guest'}
            </Typography>
            <IconButton color="inherit">
              <AccountCircleIcon />
            </IconButton>
          </Box>
        </Box>
      </Toolbar>
    </AppBar>
  );
};
