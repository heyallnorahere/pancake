# pancake

Pancake bot.
CAD drawing can be found [here](https://cad.onshape.com/documents/24af23b84924bcae2e494b27/w/7822e903a0ddc4106c8a2e95/e/12f5b05b2e40a2d157e12141).

## What is this?

This is a robot based on a "swerve" drivetrain as seen in FIRST Robotics Competition (FRC). The goal
of the project is to build a robot from the ground up, with the only caveat being that off-the-shelf
FRC parts are used exclusively for motors and electrical systems.

A characteristic of the swerve drivetrain is that the robot can move in a linear direction and
rotate at the same time. This is achieved through independent motion of all "modules" - a "module"
controls the motion of one wheel. Each module has the capability to both orient its wheel
independently all of the other wheels and also drive the wheel at the same time. This works with the
use of 2 motors per wheel - one which drives the speed of the wheel, and one which controls the
angle at which the wheel applies force. As a result of this design, the robot is able to move in a
linear direction and rotate concurrently.

Instead of using a roboRIO as robots in FRC do, this project uses a Raspberry Pi. I chose this in
order to build my own tech stack, instead of depending on the robot control systems that are
pre-built for FRC. I also used a single-board computer such as this in order to use low-level APIs
such as CAN without relying on vendor libraries to control motors and such.

This robot is not FRC-legal. The main component which disqualifies it is the custom tech stack which
I wrote on top of ROS and the Raspberry Pi.

## But why?

I started this project to create a deeper understanding of the process of designing and building a
robot. When starting on my local FRC robotics team, I was confined to the bubble of the programming
sub-group. I used this project to break out of that bubble and engage in more aspects of the
process, such as CAD, electrical systems, mechanical assembly, and the interface between user-space
robot code and electrical I/O.

I chose the object of just recreating a swerve drivetrain because it was a complex idea that would
produce a cool result. I had an intuition for how it worked when I was just working on the team, but
I wanted to find out for myself how easily it was implemented and what concessions had to be made
for practicality.

## How?

### Motors

I started with getting a single motor up and running through the Linux socket CAN interface. CAN on
Linux works by opening a socket on the `canX` network, which `X` being the interface index. This
allows one to send CAN frames on the electrical CAN network.

There are many protocols through which functionality is exposed over CAN. The most common one by far
is CANopen, which is used in industry robotics and medical equipment. FRC, however, has
[their own CAN protocol](https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html).

However, this does not explain, on its own, how to communicate with one specific kind of motor
controller. Due to the proprietary nature of the REV Robotics SPARK MAX motor controllers which I
was using, the details of the CAN API which they speak is not public.

Still, I emailed REV Robotics and politely asked for details on the CAN protocol that the SPARK MAX
motor controllers spoke in. Thankfully, they obliged, and allowed me access to a spreadsheet of CAN
IDs. This allowed me to implement my own client to speak to SPARK MAX controllers over CAN.

After much writing, debugging, and emailing, I was able to write my own vendor library to control
DC/BLDC motors through SPARK MAX controllers. However, REV stipulated that I cannot share the
details of the CAN spec, so the `vendor/librevfree` submodule (the library that I wrote to interface
with the motor controllers) is not made public.

I had issues with motor controllers later down the road. I continued to commit to the library as I
was developing the robot. It's very hard to write a library and use it without finding issues at a
later point.

(Everything past this point is frankensteined together. I'm still writing.)

## Points of note
### Ground-up design of chassis

I designed the entire chassis of the robot from the ground up.

The part of this that took the longest was by far the swerve modules. My unfamiliarity with CAD
and mechanical design only worsened the mistakes that I made when designing the modules. However,
all parts (even gears!) of the swerve modules were 3D printed out of either PLA or ABS, and they
didn't take long to re-print, nor did it greatly inflate the cost of the project.

I also 3D printed the gears inside the modules out of PLA. Initially, I was skeptical of how well
the gears would hold under torque, but with 100% fill and ABS-infused PLA, they didn't show much
wear.

### Raspberry Pi 4 Model B & raw CAN frames

Additionally, I wanted to learn how CAN works myself, instead of depending on the vendor libraries
supplied for the motor controllers I was working with. In order to achieve this, I decided to use
the low-level Linux socket CAN API to send CAN frames. 

### ROS

Because this project uses a Raspberry Pi, I needed some way to communicate with a control system. I
used ROS (Robot Operating System) on the `jazzy` distro to achieve this. However, ROS is not
natively supported on any Linux distributions other than Ubuntu. I considered installing Ubuntu on
the Raspberry Pi that I mounted onto the robot, however Docker seemed like a better choice. I could
fine-grain the methods in which the robot code could interact with the host system, and I could
control for the runtime environment of the project using the shipped Docker container.

### Cross-compiling

Single-board computers such as the Raspberry Pi run on ARM architecture. This differs from the x86
architecture that most desktop, laptop, and data center machines run on. However, because I was
developing the project in C++, and ELF binaries cannot run on architectures they were not compiled
for, I was forced to cross-compile the ROS project somehow. Initially, I emulated the `arm64`
architecture during the Docker build via QEMU in a CI/CD workflow. However, each build took around
30 minutes due to the compiler overhead introduced with C++ templating, and the proliferation of
templates in the ROS C++ client library.

The more efficient approach I figured out was to segment the Docker build into two stages, `build`
and `runtime`. The build stage runs on the native architecture of the compilation machine, while the
runtime stage runs on the target architecture that the image will be built for. The project is
cross compiled in the `build` stage using compiler wizardry and
[shell scripts](scripts/buildenv.sh). After this, the binaries are copied into the runtime stage,
and the robot code can run natively on the Raspberry Pi through the Docker hypervisor.

This part took me a good few days of headache. Possibly the best argument for using a pre-built tech
stack in robotics.

![Robot](photo.png)
