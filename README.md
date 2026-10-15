# Procedural Skeletal IK Creature (Snake Reloaded)

An interactive procedural animation experiment built with Python and Pygame featuring analytical Inverse Kinematics (IK), distance-constrained spine simulation, and adaptive multi-legged locomotion.

## Features

- **2-Bone Analytical Inverse Kinematics**: Elbow and knee joint positioning using the law of cosines.
- **Dynamic Spine Constraints**: Chain IK distance constraints allowing fluid, realistic vertebrate movement.
- **Adaptive Procedural Stepping**: Legs calculate dynamic landing targets based on heading vector and anchor reach limits.
- **Glowing Skeletal Visuals**: Multi-pass rendering creating a glowing ribcage and spine aesthetic on a deep neon canvas.

## Getting Started

### Prerequisites

- Python 3.8+
- Pygame
- NumPy

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/<your-username>/<your-repo-name>.git
   cd "snake reloaded"
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Run the simulation:
   ```bash
   python snake_creature.py
   ```

## Controls

- **Mouse Cursor**: Guide the creature's head.
- **Escape / Window Close**: Exit the application.

