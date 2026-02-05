# Autonomous Vehicle Path Planning in Underground Space

## Project Background
Autonomous navigation in underground environments poses significant challenges due to
- narrow and complex spatial structures,
- limited visibility and sensor occlusion,
- strong noise in sensor data.

This project aims to design a robust path planning framework for autonomous vehicles
operating in underground spaces, with a focus on obstacle avoidance and navigation reliability.

---

## Project Overview
This project was conducted as part of a **University Innovation and Entrepreneurship Training Program**.

We focus on improving traditional sampling-based path planning algorithms by integrating
**sensor data denoising and environment perception enhancement**, targeting underground scenarios.

---

## Methodology

### 1. Path Planning Algorithm
- Adopted **Rapidly-exploring Random Tree (RRT)** as the baseline planner
- Improved planning stability and path smoothness for narrow underground spaces

### 2. Sensor Data Denoising
- Applied **L-DnCNN-based denoising** to reduce noise in sensor inputs
- Enhanced environment perception accuracy before path planning

### 3. System Integration
- Combined denoised perception results with the planning module
- Designed a complete pipeline from perception to path generation

---

## Experimental Results
Simulation-based experiments were conducted to evaluate the proposed approach.

Results show:
- Improved path planning success rate in cluttered environments
- Reduced unnecessary detours and sharper turns
- Better robustness under noisy sensor conditions

---

## Technologies Used
- Programming: Python, C/C++
- Algorithms: RRT, path optimization
- AI Techniques: DnCNN-based denoising
- Tools: Git, Linux, simulation tools

---

## Team Information
- Team size: 5 undergraduate students
- Major: Software Engineering
- Institution: Sun Yat-sen University

---

## Author
Jiang Nan  
Undergraduate, Software Engineering  
Sun Yat-sen University
