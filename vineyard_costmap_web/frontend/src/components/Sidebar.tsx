import React from 'react';
import {
  Drawer,
  List,
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
  Divider,
  Box,
  Typography,
} from '@mui/material';
import {
  Dashboard as DashboardIcon,
  Map as MapIcon,
  Image as ImageIcon,
  Settings as SettingsIcon,
  Assignment as AssignmentIcon,
  Timeline as TimelineIcon,
  Layers as LayersIcon,
} from '@mui/icons-material';
import { useLocation, useNavigate } from 'react-router-dom';

interface SidebarProps {
  open: boolean;
  onClose: () => void;
}

const menuItems = [
  {
    text: 'Dashboard',
    icon: <DashboardIcon />,
    path: '/',
  },
  {
    text: 'Costmap Viewer',
    icon: <MapIcon />,
    path: '/costmap',
  },
  {
    text: 'Satellite Images',
    icon: <ImageIcon />,
    path: '/images',
  },
  {
    text: 'Processing Jobs',
    icon: <TimelineIcon />,
    path: '/jobs',
  },
  {
    text: 'Missions',
    icon: <AssignmentIcon />,
    path: '/missions',
  },
  {
    text: 'Layer Manager',
    icon: <LayersIcon />,
    path: '/layers',
  },
  {
    text: 'Parameters',
    icon: <SettingsIcon />,
    path: '/parameters',
  },
];

export const Sidebar: React.FC<SidebarProps> = ({ open, onClose }) => {
  const location = useLocation();
  const navigate = useNavigate();

  const handleNavigation = (path: string) => {
    navigate(path);
    if (window.innerWidth < 960) {
      onClose();
    }
  };

  const drawerContent = (
    <Box sx={{ width: 250 }}>
      <Box sx={{ p: 2 }}>
        <Typography variant="h6" color="primary">
          Vineyard Navigation
        </Typography>
        <Typography variant="body2" color="text.secondary">
          Costmap Management System
        </Typography>
      </Box>
      
      <Divider />
      
      <List>
        {menuItems.map((item) => (
          <ListItem key={item.text} disablePadding>
            <ListItemButton
              selected={location.pathname === item.path}
              onClick={() => handleNavigation(item.path)}
            >
              <ListItemIcon>
                {item.icon}
              </ListItemIcon>
              <ListItemText primary={item.text} />
            </ListItemButton>
          </ListItem>
        ))}
      </List>
    </Box>
  );

  return (
    <Drawer
      variant="temporary"
      anchor="left"
      open={open}
      onClose={onClose}
      ModalProps={{
        keepMounted: true, // Better open performance on mobile
      }}
      sx={{
        '& .MuiDrawer-paper': {
          boxSizing: 'border-box',
          width: 250,
        },
      }}
    >
      {drawerContent}
    </Drawer>
  );
};
