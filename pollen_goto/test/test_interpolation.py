import numpy as np
import pytest
from pollen_goto.interpolation import sinusoidal, JointSpaceInterpolationMode

def test_sinusoidal_basic():
    """Test basic functionality of sinusoidal interpolation."""
    # Test with single dimension
    start_pos = np.array([0.0])
    goal_pos = np.array([1.0])
    duration = 1.0
    
    # Create interpolation function
    interp_func = sinusoidal(start_pos, goal_pos, duration)
    
    # Test start position
    assert np.allclose(interp_func(0.0), start_pos)
    
    # Test end position
    assert np.allclose(interp_func(1.0), goal_pos)
    
    # Test middle position
    middle = interp_func(0.5)
    assert middle.shape == start_pos.shape
    assert np.all(middle >= start_pos) and np.all(middle <= goal_pos)

def test_sinusoidal_multi_dimension():
    """Test sinusoidal interpolation with multiple dimensions."""
    # Test with multiple dimensions
    start_pos = np.array([0.0, 0.0, 0.0])
    goal_pos = np.array([1.0, 2.0, 3.0])
    duration = 1.0
    
    # Create interpolation function
    interp_func = sinusoidal(start_pos, goal_pos, duration)
    
    # Test start position
    assert np.allclose(interp_func(0.0), start_pos)
    
    # Test end position
    assert np.allclose(interp_func(1.0), goal_pos)
    
    # Test middle position
    middle = interp_func(0.5)
    assert middle.shape == start_pos.shape
    assert np.all(middle >= start_pos) and np.all(middle <= goal_pos)

def test_sinusoidal_smoothness():
    """Test that the interpolation is smooth."""
    start_pos = np.array([0.0])
    goal_pos = np.array([1.0])
    duration = 1.0
    
    # Create interpolation function
    interp_func = sinusoidal(start_pos, goal_pos, duration)
    
    # Test smoothness by checking multiple points
    times = np.linspace(0, 1, 100)
    positions = np.array([interp_func(t) for t in times])
    
    # Check that there are no sudden jumps
    differences = np.diff(positions, axis=0)
    assert np.all(np.abs(differences) < 0.1)  # No sudden jumps
    
    # Check that the path is monotonic
    assert np.all(np.diff(positions) >= 0)  # Always increasing

def test_sinusoidal_edge_cases():
    """Test edge cases of sinusoidal interpolation."""
    start_pos = np.array([0.0])
    goal_pos = np.array([1.0])
    
    # Test zero duration
    with pytest.raises(ZeroDivisionError):
        sinusoidal(start_pos, goal_pos, 0.0)
    
    # Test negative duration
    with pytest.raises(ValueError):
        sinusoidal(start_pos, goal_pos, -1.0)
    
    # Test different shaped arrays
    with pytest.raises(ValueError):
        sinusoidal(np.array([0.0, 0.0]), goal_pos, 1.0) 