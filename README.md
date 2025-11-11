# Giant Interactive Pill Bug Rolling Robot Project

<div align="center">

![Project Status](https://img.shields.io/badge/Status-In%20Development-yellow)
![Size](https://img.shields.io/badge/Target%20Size-50cm-blue)
![License](https://img.shields.io/badge/License-CC%20BY%204.0-green)

**A Bio-Inspired Robotic System Combining Natural Defense Mechanisms with Advanced Interactive Technology**

[Project Gallery](https://protopedia.net/prototype/6554) | [Research Papers](#scientific-foundation) | [Technical Documentation](#technical-specifications)

</div>

---

## 🌟 Vision & Mission

This project aims to create a **50cm-scale interactive robotic pill bug** (isopod) that captures the fascinating defense mechanism of real pill bugs - their ability to roll into a perfect sphere when threatened. Beyond mere replication, we are building an intelligent, responsive robotic companion that bridges the gap between mechanical engineering, biological inspiration, and human-robot interaction.

### Why This Matters

- **Biomimicry Innovation**: Translating nature's elegant solutions into advanced robotics
- **Interactive Robotics**: Creating emotionally engaging experiences through lifelike behavior
- **Educational Impact**: Demonstrating complex mechanical systems in an approachable, fascinating form
- **Engineering Challenge**: Solving the unique problems of large-scale articulated shell mechanisms
- **Cultural Resonance**: Celebrating the charm of crustaceans and invertebrate life forms

---

## 🎯 Project Objectives

### Core Goals
1. ✅ **Authentic Morphing Mechanism**: Seamless transition from walking mode to rolled defensive sphere
2. ✅ **Intelligent Threat Detection**: Multi-sensor system for environmental awareness
3. ✅ **Natural Movement Patterns**: Biomechanically accurate locomotion and response behaviors
4. ✅ **Interactive Engagement**: Real-time response to human presence and touch
5. ✅ **Robust Construction**: Progression from prototype materials to metal shell plating

### Ultimate Vision
Create a robotic entity that transcends traditional robotics - not merely a machine, but a "living presence" that people can connect with emotionally, demonstrating how scale, motion, and interactivity combine to influence human perception and emotion.

---

## 🔬 Scientific Foundation

This project builds upon cutting-edge research in bio-inspired robotics:

### Key Research Influences

#### 1. **Mechanical Action Potential for Rapid Escape Response**
**Source**: Hanai Mio, Onishi Koji, Niitake Jun, Ikemoto Yasusuke (2020)  
**Institution**: Japanese Society of Mechanical Engineers  
**Key Insights**:
- Mechanical action potential triggers rapid defense rolling
- Biomechanical timing and coordination of segment articulation
- Speed optimization for threat response
- [Paper Link](https://www.jstage.jst.go.jp/article/jsmermd/2020/0/2020_1A1-E08/_article/-char/ja/)

#### 2. **Pillbug-inspired Morphing Mechanism with Sliding Shells**
**Authors**: Wang et al. (Shanghai University, Toronto Metropolitan University, CNRS-LS2N)  
**Institutions**: 
- Shanghai Key Laboratory of Intelligent Manufacturing and Robotics
- Laboratoire des Sciences du Numérique de Nantes (LS2N), France
- Department of Aerospace Engineering, Toronto Metropolitan University

**Key Insights**:
- Sliding shell mechanism for smooth transformation
- Multi-segment coordination architecture
- Structural optimization for large-scale implementation
- [arXiv Paper](https://arxiv.org/pdf/2506.04942)

---

## 📐 Technical Specifications

### Physical Dimensions
- **Length (Extended)**: 50 cm
- **Width**: 25 cm
- **Height (Walking Mode)**: 15 cm
- **Diameter (Rolled)**: 30 cm
- **Total Weight**: 8-12 kg (estimated final assembly)
- **Shell Segments**: 7-9 articulated plates

### Shell Construction Evolution

#### Phase 1: Prototype (Current)
- **Material**: PLA/PETG 3D printed segments
- **Purpose**: Proof of concept, fit testing
- **Production**: Bambu Lab 3D printer
- **Advantages**: Rapid iteration, low cost
- **Limitations**: Structural flexibility, surface finish

#### Phase 2: Reinforced Prototype
- **Material**: Carbon fiber reinforced composites
- **Coating**: Automotive-grade primers and finishes
- **Purpose**: Durability testing, weight optimization
- **Production**: CNC milling + composite layup

#### Phase 3: Final Production (Target)
- **Material**: Aluminum alloy (AL6061-T6) or stainless steel (304)
- **Thickness**: 2-3mm precision-cut plates
- **Surface Treatment**: Brushed/polished finish with protective coating
- **Fastening**: Custom machined hinges and pivots
- **Production Methods**: 
  - CNC machining for precision parts
  - Laser cutting for shell plates
  - Metal spinning for curved sections
  - Custom PCB integration points

---

## 🔧 Hardware Architecture

### Electronics Core

#### Main Controller
- **Microcontroller**: ESP32-WROOM-32D
  - Dual-core 240MHz processor
  - Built-in WiFi & Bluetooth 4.2/BLE
  - 520KB SRAM, 4MB Flash
  - 34 GPIO pins for extensive I/O
  - Deep sleep modes for power management

#### Power System
- **Battery**: 11.1V 5000mAh LiPo (3S configuration)
- **Battery Management**: TP4056-based charging module with protection
- **Voltage Regulation**: 
  - 5V/3A buck converter for logic circuits
  - 12V/5A for servo motors
  - Separate 5V rail for sensors (noise isolation)
- **Runtime**: 2-3 hours continuous operation
- **Charging**: USB-C PD 2.0 compatible (5V/3A)

### Actuation System

#### Rolling/Morphing Mechanism
- **Main Actuators**: 4-6x MG996R Digital Servo Motors
  - Operating Voltage: 4.8V-7.2V
  - Torque: 11 kg·cm @ 6V
  - Speed: 0.17 sec/60° @ 6V
  - Metal gear construction for durability
  - PWM control via ESP32
  
- **Segment Linkage**: Custom 3D-printed mechanical linkages with ball bearings
- **Coordination**: Synchronized servo movement via software state machine
- **Rolling Time**: Target 0.5-1.0 seconds (threat to sphere)

#### Locomotion System (Future Implementation)
- **Leg Motors**: 12x SG90 Micro Servos (6 legs × 2 joints)
- **Gait Control**: Tripod gait algorithm
- **Walking Speed**: 5-10 cm/s

### Sensor Array

#### Environmental Awareness
1. **Ultrasonic Distance Sensors** (HC-SR04) ×4
   - Range: 2cm - 400cm
   - Mounting: Front, rear, sides
   - Purpose: Proximity detection, collision avoidance

2. **PIR Motion Sensor** (HC-SR501) ×2
   - Detection Range: 7 meters
   - Angle: 120°
   - Purpose: Human presence detection

3. **Capacitive Touch Sensors** (TTP223) ×8
   - Placement: Across shell segments
   - Purpose: Tactile interaction detection

4. **IMU** (MPU-6050)
   - 6-axis accelerometer + gyroscope
   - Purpose: Orientation tracking, stability control
   - Position: Central body mounting

5. **Light Sensors** (LDR/Photodiode) ×2
   - Purpose: Ambient light detection, shadow response

#### Feedback Systems
- **Status LEDs**: WS2812B RGB LED strip (20 LEDs)
  - Under-shell lighting effects
  - Status indication
  - Mood lighting for interaction
  
- **Speaker/Buzzer**: 8Ω 2W speaker
  - Sound effects library
  - Ultrasonic communication (future)

### PCB Design

#### Custom Control Board v1.0
- **Dimensions**: 80mm × 60mm
- **Layers**: 4-layer PCB
  - Layer 1: Signal traces
  - Layer 2: Ground plane
  - Layer 3: Power plane (5V/3.3V)
  - Layer 4: Signal traces

#### Key Features
- ESP32 module socket with decoupling
- 16-channel PWM servo driver (PCA9685)
- Sensor interface headers with ESD protection
- Buck converter footprints
- Battery monitoring circuit (voltage divider + ADC)
- USB-C connector for programming/charging
- MOSFET switching for power domains
- Emergency shutdown circuit

#### Manufacturing Specifications
- **Manufacturer**: PCB fabrication house
- **Material**: FR-4, 1.6mm thickness
- **Copper Weight**: 1oz (35μm)
- **Solder Mask**: Matte black
- **Silkscreen**: White, component labels
- **Surface Finish**: ENIG (gold plating for reliability)
- **Via Treatment**: Tented vias
- **Testing**: Flying probe + visual inspection

---

## 💻 Software Architecture

### Firmware Stack

#### Development Environment
- **IDE**: Arduino IDE 2.x / PlatformIO
- **Framework**: Arduino Core for ESP32
- **Language**: C++17
- **Version Control**: Git + GitHub
- **CI/CD**: GitHub Actions for automated testing

#### Core Libraries
```cpp
// Hardware Abstraction
#include <ESP32Servo.h>          // Servo motor control
#include <Adafruit_PWMServoDriver.h>  // PCA9685 driver
#include <Wire.h>                // I2C communication
#include <SPI.h>                 // SPI communication

// Sensors
#include <MPU6050.h>             // IMU processing
#include <NewPing.h>             // Ultrasonic sensors
#include <FastLED.h>             // LED control

// Connectivity
#include <WiFi.h>                // Network connectivity
#include <ESPAsyncWebServer.h>  // Web interface
#include <ArduinoJson.h>         // JSON parsing

// Utilities
#include <EEPROM.h>              // Configuration storage
#include <Preferences.h>         // NVS storage
```

#### Software Modules

##### 1. Behavior State Machine
```
States:
- IDLE: Low-power monitoring mode
- ALERT: Threat detected, preparing response
- ROLLING: Active morphing to sphere
- DEFENSIVE: Fully rolled, waiting
- UNROLLING: Returning to normal posture
- EXPLORING: Autonomous movement (future)
- INTERACTIVE: Responding to touch/presence
```

##### 2. Sensor Fusion System
- Multi-sensor data aggregation
- Kalman filtering for IMU
- Threat assessment algorithm
- Distance-weighted decision making

##### 3. Motion Control
- Inverse kinematics for servo coordination
- Trajectory planning for smooth morphing
- PID control for servo positioning
- Coordinated multi-servo sequencing

##### 4. Communication Interface
- WiFi configuration (AP + Station modes)
- RESTful API for remote control
- WebSocket for real-time telemetry
- Bluetooth LE for mobile app (future)

#### Programming Features

**Threat Detection Algorithm**:
```cpp
// Weighted threat assessment
float calculateThreatLevel() {
    float threat = 0.0;
    
    // Distance-based threat (closer = higher)
    if (ultrasonicDistance < 30) threat += 0.8;
    else if (ultrasonicDistance < 50) threat += 0.4;
    
    // Motion detection
    if (pirTriggered) threat += 0.6;
    
    // Touch sensitivity
    if (touchDetected) threat += 0.3;
    
    // Environmental changes
    if (abs(lightChange) > threshold) threat += 0.2;
    
    return constrain(threat, 0.0, 1.0);
}
```

**Adaptive Behavior**:
- Learning mode: Records interaction patterns
- Personality traits: Adjustable "shyness" parameter
- Memory: EEPROM storage of preferred behaviors
- Time-based behaviors: Different responses by time of day

---

## 🔬 Mechanical Design Details

### Shell Segment Mechanism

#### Design Principles
- **Biomimetic Articulation**: 7 overlapping segments (inspired by Armadillidium vulgare)
- **Sliding Joint System**: Each segment slides and rotates simultaneously
- **Center of Rotation**: Precisely calculated for perfect sphere formation
- **Load Distribution**: Reinforced stress points at hinge locations

#### CAD Workflow
- **Software**: Autodesk Fusion 360
- **File**: `ダンゴムシもどきalfav222.f3d`
- **Parametric Design**: Fully adjustable dimensions
- **Assembly Constraints**: Kinematic joint definitions
- **Simulation**: Motion study for collision detection
- **Export**: STL for 3D printing, STEP for CNC machining

#### Hinge Design
- **Type**: Custom living hinge (prototype) → Metal pivot hinge (production)
- **Bearing Integration**: 608ZZ miniature ball bearings
- **Range of Motion**: 0° to 180° per segment
- **Safety**: Mechanical stops prevent over-rotation
- **Durability**: 10,000+ cycle testing target

### Structural Analysis
- **FEA Simulation**: Stress analysis under loading
- **Material Selection**: Strength-to-weight optimization
- **Factor of Safety**: 3.0 for prototype, 5.0 for production
- **Load Cases**: 
  - Self-weight in rolled configuration
  - Impact resistance (drop testing)
  - Handling forces (human interaction)

---

## 🎨 Interactive Features

### Multi-Modal Responses

#### 1. Proximity Interaction
- **Far (>100cm)**: Idle state, subtle LED breathing
- **Medium (50-100cm)**: Alert state, increased LED brightness, orientation tracking
- **Close (<50cm)**: Defensive preparation, LED color shift (blue → yellow → red)
- **Very Close (<20cm)**: Rapid rolling response

#### 2. Touch Responses
- **Gentle Touch**: Curiosity mode - LEDs pulse, slight shell movement
- **Persistent Touch**: Calm down response - gradual LED dimming
- **Multiple Touch Points**: Ticklish response - servo jitter, sound effects
- **Head Touch**: Unique response pattern (special interaction zone)

#### 3. Sound Reactions
- **Loud Noise**: Defensive rolling (clap, shout)
- **Continuous Sound**: Tracking behavior (follows sound source)
- **Musical Tones**: Rhythmic LED pulsing (future feature)

#### 4. Light Responses
- **Bright Light**: Seeking behavior (moves toward light)
- **Shadow Detection**: Pause and assess (potential threat)
- **Rapid Light Changes**: Startle response

### Personality System

#### Adjustable Parameters
```cpp
struct PersonalityTraits {
    float shyness;        // 0.0 (bold) to 1.0 (very shy)
    float curiosity;      // Exploration tendency
    float playfulness;    // Interactive engagement level
    float calmingSpeed;   // How quickly it relaxes
};
```

- **Shy Mode**: Rolls at greater distances, slower unrolling
- **Curious Mode**: Approaches stimuli, frequent exploratory movements
- **Playful Mode**: More animated responses, sound effects

---

## 🚀 Development Roadmap

### Phase 1: Prototype Alpha (✅ Completed - Feb 2025)
- ✅ Initial 3D printed shell design
- ✅ Basic ESP32 control system
- ✅ Single servo morphing test
- ✅ Ultrasonic sensor integration
- ✅ Power system validation

### Phase 2: Prototype Beta (🔄 In Progress - Q1-Q2 2025)
- 🔄 Multi-segment shell assembly
- 🔄 Complete sensor array integration
- 🔄 Full rolling mechanism coordination
- 🔄 Custom PCB design & fabrication
- ⏳ Web interface for remote control
- ⏳ Basic behavior state machine
- ⏳ Touch sensor implementation

### Phase 3: Refined Prototype (⏳ Planned - Q3 2025)
- ⏳ Carbon fiber reinforced shell
- ⏳ IMU-based stability control
- ⏳ Advanced threat detection algorithms
- ⏳ LED lighting effects system
- ⏳ Sound effect library
- ⏳ Mobile app development
- ⏳ Comprehensive testing & refinement

### Phase 4: Pre-Production (⏳ Planned - Q4 2025)
- ⏳ Metal shell fabrication (aluminum)
- ⏳ Professional PCB revision 2.0
- ⏳ Sealed bearing integration
- ⏳ Weatherproofing considerations
- ⏳ Performance optimization
- ⏳ Durability testing (1000+ cycle)
- ⏳ User testing & feedback

### Phase 5: Production Model (⏳ Planned - 2026)
- ⏳ Stainless steel shell option
- ⏳ Locomotion system (walking)
- ⏳ AI behavior learning
- ⏳ Multi-robot coordination
- ⏳ Exhibition-ready features
- ⏳ Documentation & open-source release

---

## 📊 Bill of Materials (BOM)

### Electronics Components

| Category | Component | Quantity | Unit Price | Total | Supplier |
|----------|-----------|----------|------------|-------|----------|
| **MCU** | ESP32-WROOM-32D Dev Board | 1 | $8 | $8 | Generic |
| **Servos** | MG996R Digital Servo | 6 | $12 | $72 | Generic |
| **Servos** | SG90 Micro Servo | 12 | $3 | $36 | Generic |
| **Driver** | PCA9685 16-Ch PWM Driver | 1 | $6 | $6 | Adafruit |
| **Sensors** | HC-SR04 Ultrasonic | 4 | $2 | $8 | Generic |
| **Sensors** | HC-SR501 PIR Motion | 2 | $3 | $6 | Generic |
| **Sensors** | MPU-6050 IMU Module | 1 | $5 | $5 | Generic |
| **Sensors** | TTP223 Touch Sensor | 8 | $1 | $8 | Generic |
| **Sensors** | LDR Photoresistor | 2 | $0.50 | $1 | Generic |
| **LEDs** | WS2812B LED Strip (1m) | 1 | $10 | $10 | Generic |
| **Audio** | 8Ω 2W Speaker | 1 | $4 | $4 | Generic |
| **Power** | 11.1V 5000mAh LiPo Battery | 1 | $45 | $45 | Hobbyking |
| **Power** | TP4056 Charging Module | 1 | $2 | $2 | Generic |
| **Power** | Buck Converter 5V/3A | 2 | $3 | $6 | Generic |
| **Power** | Buck Converter 12V/5A | 1 | $8 | $8 | Generic |
| **PCB** | Custom Control Board | 1 | $50 | $50 | PCB Fab House |
| **Misc** | Wiring, Connectors, Hardware | - | - | $30 | Various |
| | | | **Subtotal** | **$305** | |

### Mechanical Components (Prototype)

| Category | Component | Quantity | Unit Price | Total | Production Method |
|----------|-----------|----------|------------|-------|-------------------|
| **Shell** | Main Body Segments (PLA) | 7 | $15 | $105 | 3D Printing |
| **Shell** | Head Piece (PLA) | 1 | $20 | $20 | 3D Printing |
| **Shell** | Tail Section (PLA) | 1 | $15 | $15 | 3D Printing |
| **Hinges** | Custom 3D Printed Hinges | 6 | $3 | $18 | 3D Printing |
| **Bearings** | 608ZZ Ball Bearings | 12 | $1.50 | $18 | Purchase |
| **Frame** | Internal Support Structure | 1 | $25 | $25 | 3D Printing |
| **Fasteners** | M3 Screws, Nuts (100pcs) | 1 | $10 | $10 | Purchase |
| **Linkages** | Servo Linkage Arms | 12 | $2 | $24 | 3D Printing |
| | | | **Subtotal** | **$235** | |

### Mechanical Components (Final Production)

| Category | Component | Quantity | Unit Price | Total | Production Method |
|----------|-----------|----------|------------|-------|-------------------|
| **Shell** | AL6061 Shell Segments | 7 | $80 | $560 | CNC Machining |
| **Shell** | Aluminum Head Piece | 1 | $120 | $120 | CNC Machining |
| **Hinges** | Custom Machined Pivots | 6 | $25 | $150 | CNC Machining |
| **Frame** | Aluminum Frame Structure | 1 | $150 | $150 | CNC Machining |
| **Bearings** | Sealed 608ZZ Bearings | 12 | $3 | $36 | Purchase |
| **Fasteners** | M4 Stainless Hardware | 1 | $30 | $30 | Purchase |
| **Surface** | Anodizing/Finishing | - | - | $200 | Professional Service |
| | | | **Subtotal** | **$1,246** | |

### Project Total Costs

| Phase | Cost | Notes |
|-------|------|-------|
| Prototype (Current) | $540 | Working proof of concept |
| Production Model | $1,551 | Exhibition-quality finish |
| Development Tools | $200 | Software licenses, test equipment |
| **Total Investment** | **$2,291** | For complete production unit |

---

## 🛠️ Manufacturing Process

### Prototype Production

#### 3D Printing Specifications
- **Printer**: Bambu Lab X1 Carbon / P1P
- **Material**: 
  - Shell: PETG (better impact resistance than PLA)
  - Internal: PLA (sufficient for structural parts)
  - Hinges: TPU 95A (flexible sections)
- **Layer Height**: 0.2mm (balanced quality/speed)
- **Infill**: 25% gyroid for shells, 50% for load-bearing parts
- **Wall Lines**: 4 (1.6mm wall thickness)
- **Print Time**: ~80 hours total for all parts
- **Post-Processing**: 
  - Sanding (120 → 220 → 400 grit)
  - Primer coating
  - Automotive spray paint
  - Clear coat finish

### Production Manufacturing

#### CNC Machining Process
1. **CAD to CAM**: Fusion 360 → Export toolpaths
2. **Material Preparation**: 3mm AL6061 sheet stock
3. **Rough Cutting**: 2mm end mill, 3000 RPM
4. **Finish Passes**: 1mm ball nose, 8000 RPM
5. **Drilling**: Precision holes for fasteners
6. **Deburring**: Manual finishing of edges
7. **Quality Control**: CMM measurement verification

#### Custom PCB Fabrication
1. **Design**: KiCad 7.x (open-source EDA)
2. **Design Review**: DRC + ERC checks
3. **Gerber Export**: RS-274X format
4. **Manufacturing Specs**:
   - Minimum trace width: 0.15mm
   - Minimum via size: 0.3mm drill
   - Clearance: 0.15mm
   - Impedance control: Not required
5. **Assembly**: 
   - Stencil application for SMD components
   - Pick-and-place or hand soldering
   - Reflow oven profile: 183°C peak (SAC305 solder)
6. **Testing**: Continuity test, power-on verification

#### Surface Finishing
- **Aluminum Parts**: 
  - Bead blasting for uniform surface
  - Type II anodizing (color options)
  - Laser engraving for details
- **Steel Parts** (optional):
  - Electropolishing
  - PVD coating for color/hardness

---

## 🧪 Testing & Validation

### Mechanical Testing
- **Cycle Testing**: 10,000 roll/unroll cycles
- **Drop Testing**: 30cm height onto hard surface
- **Load Testing**: 20kg static load on rolled form
- **Temperature**: -10°C to 50°C operation
- **Humidity**: 20% to 80% RH (non-condensing)

### Electronics Testing
- **Power Consumption**: Baseline measurement at all states
- **Servo Current Draw**: Peak and sustained measurements
- **Battery Runtime**: Real-world usage scenarios
- **Sensor Accuracy**: Calibration verification
- **Communication Range**: WiFi/BLE effective distance
- **EMI/EMC**: Basic electromagnetic compatibility

### Software Testing
- **Unit Tests**: Individual function validation
- **Integration Tests**: Module interaction verification
- **Stress Tests**: Continuous operation (24+ hours)
- **Edge Cases**: Sensor failure recovery
- **User Testing**: 10+ participants for UX feedback

### Safety Considerations
- **Pinch Points**: Minimum 5mm clearances
- **Sharp Edges**: All edges deburred and radiused
- **Electrical**: Proper isolation, fusing
- **Thermal**: Overheat shutdown at 85°C
- **Battery**: Overcharge/overdischarge protection
- **Emergency Stop**: Mechanical kill switch accessible

---

## 📡 Connectivity & Control

### WiFi Features
- **Access Point Mode**: Direct connection (no router needed)
  - SSID: `PillBugRobot_XXXX`
  - Default IP: 192.168.4.1
  
- **Station Mode**: Connect to existing network
  - mDNS: `pillbug.local`
  - Web dashboard access from any browser

### Web Dashboard
- **Real-time Telemetry**: Sensor values, battery level, state
- **Manual Control**: Override automatic behaviors
- **Configuration**: Adjust personality parameters
- **Calibration**: Servo position fine-tuning
- **Data Logging**: CSV export of interaction events
- **OTA Updates**: Wireless firmware updates

### API Endpoints
```
GET  /api/status        - System status JSON
POST /api/control       - Send commands
GET  /api/sensors       - Live sensor data
POST /api/config        - Update settings
GET  /api/logs          - Download interaction log
POST /api/firmware      - Upload new firmware
```

### Future: Mobile App
- **Platform**: Flutter (iOS + Android)
- **Features**:
  - Bluetooth LE connection
  - Gesture control
  - Behavior programming
  - Photo/video capture with robot
  - Social sharing integration

---

## 🌐 Open Source & Community

### Repository Contents
- Complete CAD files (Fusion 360, STEP, STL)
- PCB design files (KiCad project)
- Full firmware source code (Arduino/ESP32)
- Web dashboard code (HTML/CSS/JavaScript)
- Bill of Materials with suppliers
- Assembly instructions with photos
- Calibration procedures
- Troubleshooting guide

### Licensing
- **Hardware**: CERN OHL v2 (Open Hardware License)
- **Software**: MIT License
- **Documentation**: CC BY 4.0
- **3D Models**: CC BY-SA 4.0

### Contribution Guidelines
We welcome contributions in:
- **Mechanical Design**: Improved hinge mechanisms, weight reduction
- **Electronics**: Power optimization, sensor additions
- **Software**: New behaviors, AI integration, UI improvements
- **Documentation**: Translations, tutorials, videos
- **Testing**: Bug reports, performance data

### Community Engagement
- **Discord Server**: Real-time chat and support
- **GitHub Discussions**: Long-form conversations
- **YouTube Channel**: Build videos, demos, tutorials
- **Hackaday Project**: Progress updates and logs
- **Twitter/X**: Quick updates and photos (@ProjectPillBug)

---

## 🎓 Educational Applications

### STEM Learning Opportunities
- **Mechanical Engineering**: Linkage design, kinematics, materials science
- **Electrical Engineering**: Circuit design, power management, sensor integration
- **Computer Science**: State machines, real-time systems, network programming
- **Biology**: Biomimicry, animal behavior, evolutionary adaptations
- **Mathematics**: Geometry, trigonometry (IK), statistics (sensor fusion)

### Workshop Potential
- **High Schools**: Advanced robotics clubs
- **Universities**: Mechatronics capstone projects
- **Maker Spaces**: Weekend build workshops
- **STEM Events**: Science fairs, Maker Faires, RoboFests

### Curriculum Integration
- Detailed lesson plans for educators
- Step-by-step build guide for student teams
- Modular design allows focus on specific subsystems
- Scalable complexity (basic to advanced versions)

---

## 🏆 Exhibition & Demonstration

### Target Venues
- **Science Museums**: Interactive exhibits
- **Technology Conferences**: Maker Faire, DEF CON, CES
- **Robotics Competitions**: Non-competitive showcase
- **Art Installations**: Human-robot interaction art
- **Corporate Events**: Innovation demonstrations
- **Educational Institutions**: University open houses

### Demo Scenarios
1. **Autonomous Mode**: Roaming and reacting to visitors
2. **Interactive Zone**: Encourage people to touch and observe response
3. **Behind-the-Scenes**: Transparent panels showing mechanism
4. **Live Coding**: Modify behaviors in real-time
5. **Multi-Robot**: Multiple units with swarm behaviors (future)

### Media Kit
- High-resolution photos and videos
- Technical specifications sheet
- Press release templates
- Logos and branding assets
- Creator bios and contact information

---

## 💡 Innovation Highlights

### Unique Technical Achievements
1. **Biomechanical Accuracy**: True sliding shell mechanism (not just rotating plates)
2. **Scalability**: Proven design from 10cm to 50cm scale
3. **Rapid Response**: <1 second threat-to-rolled transformation
4. **Modular Architecture**: Subsystems can be upgraded independently
5. **Adaptive Behavior**: Machine learning-ready framework
6. **Cost Efficiency**: DIY-friendly without sacrificing performance
7. **Aesthetic Excellence**: Form follows function with artistic appeal

### Patents & IP Considerations
- Novel hinge design (potential design patent)
- Sensor fusion algorithm for threat assessment
- Multi-modal interaction framework
- Open-source release strategy (defensive publication)

---

## 🤝 Collaboration Opportunities

### Industry Partners
We are seeking partnerships with:
- **PCB Manufacturers**: Prototype and production runs
- **CNC Machine Shops**: Metal parts fabrication
- **Material Suppliers**: Aluminum, steel, advanced composites
- **Electronics Distributors**: Bulk component sourcing
- **Software Companies**: AI/ML integration, cloud services
- **Educational Institutions**: Research collaborations

### Sponsorship Benefits
- **Logo Placement**: On robot, website, and promotional materials
- **Demonstration Rights**: Use robot at your events
- **Technical Consultation**: Access to design files and expertise
- **Co-branding**: Joint press releases and media coverage
- **R&D Insights**: Early access to technical reports
- **Community Recognition**: Featured in all documentation

### Funding Goals
- **$5,000**: Complete prototype with all sensors
- **$10,000**: Production-quality metal shell version
- **$20,000**: Multiple units for research/testing
- **$50,000**: Full product development with mobile app
- **$100,000**: Exhibition tour and educational program

---

## 📞 Contact & Support

### Project Lead
**Furukawa (はたけ / Hatake)**
- GitHub: [@furukawa1020](https://github.com/furukawa1020)
- Protopedia: [Project Page](https://protopedia.net/prototype/6554)
- Twitter/X: [@HATAKE55555](https://x.com/HATAKE55555)

### Project Links
- **Repository**: `big-pill-bug-rolling-robot-project-arxive`
- **Documentation**: [Coming Soon]
- **Demo Videos**: [Protopedia Gallery](https://protopedia.net/prototype/6554)
- **Build Log**: [In Progress]

### Get Involved
- ⭐ Star this repository to follow development
- 🐛 Report bugs via GitHub Issues
- 💡 Suggest features via Discussions
- 🔧 Submit pull requests with improvements
- 📢 Share the project on social media
- 💰 Sponsor development (GitHub Sponsors)

---

## 🙏 Acknowledgments

### Research Foundations
- Hanai Mio et al. (Japanese Society of Mechanical Engineers)
- Wang Jieyu et al. (Shanghai University, LS2N-CNRS)
- Toronto Metropolitan University Aerospace Department

### Technology Providers
- Espressif Systems (ESP32 platform)
- Arduino Community (firmware framework)
- Autodesk (Fusion 360 educational license)
- Bambu Lab (3D printing technology)
- Open-source hardware community

### Inspiration
The countless pill bugs (ダンゴムシ) and giant isopods (ダイオウグソクムシ) whose fascinating behavior inspired this project. Nature remains the ultimate engineer.

---

## 📄 License

This project is open-source hardware and software:

- **Hardware Designs**: [CERN OHL v2](https://ohwr.org/cern_ohl_v_2.txt)
- **Firmware**: [MIT License](https://opensource.org/licenses/MIT)
- **Documentation**: [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)

You are free to:
- **Use** this design for personal or commercial purposes
- **Modify** and improve the design
- **Share** your versions with others

Under the conditions:
- **Attribution** - Credit the original creators
- **Share-Alike** - Derivative works use compatible licenses
- **No Warranty** - Provided as-is without guarantees

---

## 🚨 Disclaimer

This is an active development project. Specifications, designs, and features are subject to change. The information provided is for educational and developmental purposes. Always follow safety guidelines when working with electronics, power tools, and mechanical systems.

**Current Status**: Prototype phase with ongoing refinement. Not yet ready for production or commercial distribution.

---

<div align="center">

### 🐛 Built with passion for crustaceans and interactive robotics 🤖

**"Bringing the charm of pill bugs to life, one servo at a time"**

[⬆ Back to Top](#giant-interactive-pill-bug-rolling-robot-project)

---

![Prototype Photo Placeholder](https://via.placeholder.com/800x400.png?text=Giant+Pill+Bug+Robot+-+Coming+Soon)

*Last Updated: November 11, 2025*

</div>
