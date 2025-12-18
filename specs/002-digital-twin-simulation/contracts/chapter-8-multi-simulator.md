# Chapter 8: Gazebo–Unity Multi-Simulator Pipeline

**Priority**: P4 (Capstone chapter integrating all concepts)
**Estimated Duration**: 2-3 hours
**Target Audience**: Readers who completed Chapters 5-7

## Learning Objectives

1. **Understand** Gazebo-Unity data flow architecture
2. **Create** launch files coordinating both simulators
3. **Implement** synchronization strategies for pose and sensor data
4. **Optimize** performance to achieve 30+ FPS and <100ms latency
5. **Troubleshoot** common synchronization and performance issues

## Chapter Structure

### 8.1 Multi-Simulator Architecture (25 min)
- Data flow diagram (Gazebo → ROS 2 → Unity)
- Component responsibilities (physics vs rendering)
- Message passing overview
- Synchronization challenges

### 8.2 Launch File Coordination (30 min)
- Combined launch file starting Gazebo + Unity bridge
- Initialization order and timing
- Parameter configuration
- Node dependencies

### 8.3 Synchronization Strategies (35 min)
- Joint state synchronization (sensor_msgs/JointState)
- TF tree synchronization (tf2_msgs/TFMessage)
- Sensor data forwarding (images, lidar)
- Timestamping and interpolation

### 8.4 Performance Monitoring and Optimization (30 min)
- Latency measurement tools
- Frame rate monitoring (Gazebo and Unity)
- Network bandwidth analysis
- Optimization techniques (message rate reduction, LOD)

### 8.5 Troubleshooting Multi-Simulator Issues (25 min)
- Connection failures
- High latency (>200ms)
- Desynchronization (Unity lagging behind Gazebo)
- Performance bottlenecks

### 8.6 Hands-On Exercises (30-45 min)
- Exercise 8.1: Launch complete pipeline
- Exercise 8.2: Measure and optimize latency
- Exercise 8.3: Custom robot in dual-simulator setup

## Key Deliverables

1. `launch_both_simulators.launch.py` - Master launch file
2. `latency_measurement.py` - Latency analysis script
3. `performance_monitor.py` - Real-time performance dashboard

## Validation Checklist

- [ ] Both simulators launch successfully via single command
- [ ] Robot state syncs within 100ms
- [ ] Gazebo runs at >100 Hz physics rate
- [ ] Unity renders at >30 FPS
- [ ] 10-minute stability test passes without crashes
