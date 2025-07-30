import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { ThemeProvider, createTheme } from '@mui/material/styles';
import { CssBaseline, Box } from '@mui/material';
import { Header } from './components/Header';
import { Sidebar } from './components/Sidebar';
import { Dashboard } from './pages/Dashboard';
import { CostmapPage } from './pages/CostmapPage';
import { useAppStore } from './hooks/useAppStore';
import { useROS } from './hooks/useROS';

const theme = createTheme({
  palette: {
    mode: 'light',
    primary: {
      main: '#2e7d32',
    },
    secondary: {
      main: '#ff9800',
    },
  },
});

function App() {
  const { sidebarOpen, setSidebarOpen } = useAppStore();
  
  // Initialize ROS connection
  useROS();

  const handleMenuClick = () => {
    setSidebarOpen(!sidebarOpen);
  };

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <Router>
        <Box sx={{ display: 'flex', flexDirection: 'column', height: '100vh' }}>
          <Header onMenuClick={handleMenuClick} />
          
          <Box sx={{ display: 'flex', flex: 1, overflow: 'hidden' }}>
            <Sidebar open={sidebarOpen} onClose={() => setSidebarOpen(false)} />
            
            <Box sx={{ flex: 1, overflow: 'auto' }}>
              <Routes>
                <Route path="/" element={<Dashboard />} />
                <Route path="/costmap" element={<CostmapPage />} />
                <Route path="/images" element={<div>Satellite Images Page (TODO)</div>} />
                <Route path="/jobs" element={<div>Processing Jobs Page (TODO)</div>} />
                <Route path="/missions" element={<div>Missions Page (TODO)</div>} />
                <Route path="/layers" element={<div>Layer Manager Page (TODO)</div>} />
                <Route path="/parameters" element={<div>Parameters Page (TODO)</div>} />
              </Routes>
            </Box>
          </Box>
        </Box>
      </Router>
    </ThemeProvider>
  );
}

export default App;
