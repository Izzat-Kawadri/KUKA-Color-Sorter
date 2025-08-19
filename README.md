# 🎨 KUKA Color Sorter: Two-Robot Collaborative Cube Sorting Simulation

![kukaRobot](./images/kukaRobot.jpg)

A **two-robot collaborative simulation** where KUKA robotic arms detect colored cubes, maintain the color order, and place cubes in the correct arenas using **Webots** and **Python**.

---

## 🌟 Simulation Scenario

This project simulates a **two-robot collaborative color sorting system** with a structured environment:

### Environment Layout
- **🟢 Zone 1:** Workspace for **Robot A** – scans and collects cubes.
- **🔵 Zone 2:** Workspace for **Robot B** – waits for cubes delivered by Robot A.
- **🧱 Wall:** Divides the zones.
- **🎨 Color Arenas:** Four target areas for cube placement:
  - 🟦 Blue Arena
  - 🟥 Red Arena
  - 🟨 Yellow Arena
  - 🟩 Green Arena

---

### Robot Behavior
1. **🤖 Robot A – The Color Scanner & Collector:**
   - Scans cubes in its zone to determine the **color order**.
   - Stores the order in a **color array**.
   - Picks cubes sequentially according to the array.
   - Places cubes on the wall for Robot B to access.  

2. **🤖 Robot B – The Secondary Collector:**
   - Scans the color array to know **where each cube should go**.
   - Waits in front of the wall until cubes are delivered.
   - Collects cubes from the wall.
   - Places cubes in the correct **color-coded arena**, maintaining order.

---

### Key Features
- 🤝 **Collaborative Robotics:** Two robots working together to sort cubes.
- 🎨 **Color Recognition & Order Management:** Scans cubes and maintains order arrays.
- ⚙️ **Autonomous Motion:** Robots pick, transfer, and place cubes automatically.
- 🏗️ **Structured Environment:** Two zones separated by a wall for organized operation.


## 📝 How It Works

    Robot A scans cubes, stores the color order, and delivers them to the wall.

    Robot B scans the color array, waits at the wall, collects cubes, and places them in the correct arenas:

        🟦 Blue

        🟥 Red

        🟨 Yellow

        🟩 Green

    The process repeats until all cubes are sorted correctly.
    

## ⚠️ Project Status

This project is a **work in progress** and still requires further development .

**Note:** The current code needs more development to **optimize the simulation** and incorporate **best programming practices** for readability, maintainability, and performance.

It serves as a **foundation for collaborative robotics simulations** and will be enhanced in future versions.


---

## 📂 Project Structure

```
KUKA-Color-Sorter/
│
├── README.md                      # Project documentation
├── world.wbt                      # Webots simulation world
├── controllers/
│ └── my_controller_robot1/
│ ├── my_controller_robot1.py      # Main Python controller
│ └── my_controller_robot2/
│ ├── my_controller_robot2.py      # Main Python controller
```

## ⚙️ Setup Instructions
1. Install [Webots](https://cyberbotics.com/).
2. Open `world.wbt` in Webots.
3. Click Run Simulation

---

## 🎬 Demo Video

You can watch a **demo video** of the current simulation in action:

[Watch Demo](./images/demo.mp4)


## 📸 Screenshots

![Screenshot](./images/Screenshot.mp4)


## 🎯 Future Improvements

Implement inverse kinematics for smoother motion.

Support multiple cubes simultaneously.

Enhance color detection with advanced image processing.


## 📜 License

MIT License – freely use, modify, and distribute.

---
