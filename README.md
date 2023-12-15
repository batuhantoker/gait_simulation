# Gait Simulation 

## Description
This project is a Python-based gait simulation tool that integrates the `slip_control` package with a `tkinter` graphical user interface. It aims to simulate and visualize gait dynamics using a 3D model, offering a unique platform for biomechanical and robotic research.

## Features
- Real-time 3D gait simulation
- Interactive GUI built with tkinter for easy parameter adjustments
- Integration with slip_control for advanced biomechanical modeling
- Ability to save animations of the gait simulation

## Installation
Ensure you have Python installed on your system. To install the necessary dependencies, run the following command in your terminal:

```bash
pip install matplotlib numpy scipy tkinter slip_control
```
## Usage

To start the application, navigate to the project directory and run:

```bash
python gait_simulation.py
```
Upon running the application, a GUI will appear where you can:

Adjust parameters like height, number of cycles, theta, and torso width
Run the simulation to see real-time gait dynamics
Save the animation by entering a filename and clicking the "Save animation" button

## Contributing
Contributions to the project are welcome. Please follow these steps:

1. Fork the repository
2. Create a new branch (`git checkout -b feature-branch`)
3. Commit your changes (`git commit -am 'Add some feature'`)
4. Push to the branch (`git push origin feature-branch`)
5. Open a pull request

## Acknowledgments
Based on the [slip_control package](https://github.com/Danfoa/slip_control).
