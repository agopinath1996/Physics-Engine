# Physics-Engine
Custom Physics Engine Implementation

## Rigid body
## Collisions
## Simulation
## Rules

```mermaid
flowchart LR
    H(Scene Modelling)==>A
    A[Rigid Body
    Modelling] ==>dbE[("`RigidBody`")]==>|Broadphase| paraB[/"`Collision Detection 
    (GJK + EPA)`"/] ==> paraC[/"`Force generation and resolution`"/]
     ==>paraD[/"`Error Correction`"/]==>|Generate new frame|dbE
```
