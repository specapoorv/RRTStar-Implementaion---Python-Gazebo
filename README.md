# 🗺️ Optimal Path Planning: RRT* Implementation in Gazebo

An algorithmic implementation of the **Rapidly-exploring Random Tree Star (RRT*)** algorithm for autonomous navigation, developed in Python and simulated in Gazebo. 

## 🎯 Project Overview
While standard RRT finds a valid path through obstacles, RRT* guarantees asymptotic optimality by continuously "rewiring" the tree to find the shortest possible route. This repository bridges the gap between pure algorithmic theory and robotic simulation by deploying the RRT* path planner within a 3D Gazebo environment, demonstrating collision avoidance and trajectory optimization.

## 🧠 The Algorithm (RRT*)
This implementation focuses on navigating continuous configuration spaces efficiently:
1. **Random Sampling:** Generates random nodes within the defined boundaries of the Gazebo world.
2. **Steer & Collision Check:** Grows the tree toward the sampled node while verifying the path doesn't intersect with any physical obstacles.
3. **Cost Optimization & Rewiring (The "*" advantage):** Evaluates nodes within a search radius to find the lowest-cost parent. It then rewires existing branches if routing them through the new node yields a shorter total distance to the start.

## 🔄 System Architecture
The following diagram illustrates how the Python algorithmic node interacts with the Gazebo simulation:

```mermaid
graph TD
    subgraph "Python Path Planner (RRT*)"
        Sample[Sample Space] --> Nearest[Find Nearest Node]
        Nearest --> Steer[Steer & Check Collision]
        Steer --> Rewire[Rewire Tree]
        Rewire --> Output[Generate Waypoints]
    end
    
    subgraph "Gazebo Simulation"
        Output -->|Trajectory Commands| Robot[Robot Model]
        Robot -->|Sensor Data / Map| Sample
    end
    
    classDef algo fill:#2d3436,stroke:#0984e3,stroke-width:2px,color:#fff;
    classDef sim fill:#00b894,stroke:#00cec9,stroke-width:2px,color:#fff;
    
    class Sample,Nearest,Steer,Rewire,Output algo;
    class Robot sim;
