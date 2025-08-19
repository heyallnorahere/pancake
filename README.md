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
### Spinning a motor
#### SOLO

I chose a Raspberry Pi to control the robot for several reasons. It's cheap, supports a widely-used
operating system, runs on very little power (<1 amp @ 5V), and it does not require an SDK or
specialized development environment to develop for it. While researching the requirements for this
project, however, it was very difficult to find any motor controllers that fell within the budget of
this project while also being guaranteed to work on any CAN-supported computer.

The best candidate I could find that fit both of these criteria was
[SOLO Motor Controllers](https://solomotorcontrollers.com), based out of Europe. Specifically, their
UNO v2 controller with its 45A max current. The only issue was that one controller costed 239 euros
(around 278 USD). Despite the cost, however, this was still the cheapest option that was not
confined to a proprietary ecosystem. I ordered 2 controllers for testing and development purposes so
that I could design and build a module while waiting for more to arrive.

SOLO controllers communicate with their controlling devices via CANopen, among other supported
protocols. I decided to stick with a CAN protocol due to its proliferation in FRC. SOLO provides
a library in both Python and C++ to communicate with their controllers. I was drawn to using their
C++ API due to my familiarity with native programming. However, the C++ library was written locked
to Windows, and my device of choice was running a distrobution of Linux. It's not very feasible to
install Windows on a Raspberry Pi, nor was embedding Windows an attractive option from a familiarity
perspective. I had an Intel Nuc that I had installed Windows on, however it was not common to attach
a CAN interface to a computer through USB, and I was also just aversed to Windows. I had already
bought a CAN hat for the Raspberry Pi, so the Python API seemed like my best option, as
uncomfortable as I was with the language.

#### NEO Vortex & NEO 550 motors

The most common type motor in FRC is **brushless DC** (BLDC). These motors generally have high
torque and top speed. Initially, I planned to drive both the wheel's speed and orientation using the
same kind of motor, the [NEO Vortex](https://revrobotics.com/rev-21-1652) from REV Robotics. I chose
this motor because it has a relatively high stall torque, and it is widely used in FRC.

This motor is designed to work with either a [SPARK Flex](https://revrobotics.com/rev-11-2159)
controller or a [Solo Adapter](https://revrobotics.com/rev-11-2828) together with a
[SPARK MAX](https://revrobotics.com/rev-11-2158) controller. Due to my previous initial decision of
using a SOLO controller, I decided to buy 2 of both Vortex motors and Solo Adapters. Before I did
this, however, I emailed REV Robotics to verify that a Solo Adapter would work with a generic BLDC
motor controller, to which they affirmed yes. With their confirmation, I ordered the motors and
adapters.

When the Vortex motors and SOLO controllers arrived, I decided to wire one set up for testing.
I assembled the motor onto the Solo Adapter, connected the three phase wires to the SOLO controller,
and and powered the SOLO controller with an oscilloscope power supply. However, I also needed to
connect the internal encoder to the controller. The encoder allows the computer to receive feedback
from the motor about how fast it's moving, and how much it's moved since it was powered on. I cut
the 6-pin JST encoder cable that came with the Solo Adapter to half length, and soldered it to the
smaller-pitch 6-pin JST cable, using mappings for the cable found on
[REV's docs](https://docs.revrobotics.com/brushless/spark-max/specs/encoder-port) and the UNO v2
[user manual](https://solomotorcontrollers.com/wp-content/uploads/materials/SOLO_UNO_v2_SLU0722_5832_UserManual.pdf).
This allowed me to spin the motor through the controller's UART interface.

When I began designing the module, I realized that for module azimuth, I did not need
nearly as much torque AND speed as the Vortex provided. In order to minimize both power usage and
project cost, I replaced the azimuth motor with a [NEO 550](https://revrobotics.com/rev-21-1651)
motor in my design, and ordered one motor. I also connected the motor to a set of gearboxes which
decreased its gear ratio to 1:12, increasing its torque while decreasing the speed at which the
wheel rotates in relation to the motor's speed. This can be easily accounted for in code.

#### SPARK MAX

As I was noticing the corner of unfamiliarity I was backing myself into with my developing tech
stack, I decided to send an email to REV Robotics concerning the API for their proprietary motor
controller, the SPARK MAX. I was familiar with this controller due to my experience programming on
an FRC team. This seemed like an amazing alternative to SOLO due to its much cheaper cost of 90 USD.
I asked if there was a library for desktop development environments that I had missed for
communicating with their motor controllers, or if they could provide me with the CAN protocol details
if they did not have such a library published. To my surprise, they sent me a spreadsheet describing
the details of SPARK MAX CAN frames. This gave me a viable alternative to the much more expensive
SOLO controllers.

After careful consideration, I decided to substitute the SOLO controllers for SPARK MAXs. I scrapped
the Python code that I had written to communicate with the controllers, and I ordered 1 SPARK MAX,
learning from my mistake previously of ordering 2 controllers to test. It was also convenient that
the controllers shipped from the US, rather than Italy.

As I now had to write my own API for CAN communication, I was able to write my new stack in a
language I was more familiar with. With Python off the table, I was opting towards C++. I briefly
considered Java, being both the most common language in FRC and also the language that my team works
in. However, I would need to waste time working with the Java Native Interface, communicating
directly with the Linux kernel. This seemed like too much of a hassle in a language that was both
abstracted from low-level APIs and not my language of choice, so I decided to stick with my comfort
language: C++.

CAN on Linux works by opening a socket on the `canX` network, with `X` being the interface index.
This allows one to send CAN frames on the electrical CAN network via the ubiquitous Linux socket
API. Through Googling I came across this great
[document](https://kernel.org/doc/Documentation/networking/can.txt) which describes how the socket
CAN interface is used. This greatly sped up the software development process.

There are many protocols through which functionality is exposed over CAN. The most common one by far
is CANopen, which is used in industry robotics and medical equipment. FRC, however, has
[their own CAN protocol](https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html).
Even so, this specification does not explain on its own how to communicate with one specific kind of
motor controller. This in combination with the spreadsheet that REV support shared with me made it
possible for me to begin working on my own library to communicate with SPARK MAX controllers.

After much writing, debugging, and emailing, I was able to write my own vendor library to control
DC/BLDC motors through SPARK MAX controllers. However, REV stipulated that I cannot share the
details of the CAN spec, so the `vendor/librevfree` submodule (the library that I wrote to interface
with the motor controllers) is not made public.

I had issues with motor controllers later down the road. Accordingly, I continued to commit to the
library as I was developing the robot. It's very hard to write your own library and use it without
finding issues.

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
