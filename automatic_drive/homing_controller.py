# ----------------------------
# Rule-Based Homing Algorithm
# ----------------------------

import math
import numpy as np

class HomingController:
    def __init__(self):
        # Thresholds from the algorithm
        self.distance_constant = 0.0 # meters (constant since simulation measures from middle of robot
        self.obsThreshold1 = 1.2 + self.distance_constant  # meters
        self.obsThreshold2 = 0.8 + self.distance_constant  # meters
        self.sideObsThreshold = 1.0 + self.distance_constant  # meters
        self.approachThreshold = 1.0  # meters
        self.fullSpin = False
        
        # Velocity limits
        self.maxLinearVel = 5.0  # rad/s for wheels
        self.maxAngularVel = 5.0  # rad/s differential
        
        # State
        self.obstacle = False
        self.turnDir = 0  # -1 for left, 1 for right
        
    def compute_velocities(self, goalDist, goalAngle, centerReading, leftReading, rightReading, debug=False):
        if debug:
            print("\n---- HOMING CONTROLLER STEP ----")
            print(f"GoalDist={goalDist:.3f}  GoalAngle={goalAngle:.3f} rad")
            print(f"Depth Readings:  L={leftReading:.3f}  C={centerReading:.3f}  R={rightReading:.3f}")
            print(f"Prev obstacle={self.obstacle}  turnDir={self.turnDir}")
    
        goalLinearVel  = 0.0
        goalAngularVel = 0.0
    
        # Check for center obstacle conditions
        center_obstacle = (centerReading < self.obsThreshold1 and goalDist > centerReading)
    
        # ====== ORANGE BOX (Center Obstacle) ======
        if center_obstacle:
            if debug:
                print("STATE: ORANGE BOX — CENTER OBSTACLE DETECTED")
                print(f"Condition: centerReading({centerReading:.3f}) < obsThreshold1({self.obsThreshold1})")
    
            # Linear velocity handling
            if centerReading < self.obsThreshold2:
                if debug:
                    print("Close obstacle — reducing linear velocity aggressively")
                goalLinearVel = self.maxLinearVel * (centerReading - self.obsThreshold2) / (self.obsThreshold1 - self.obsThreshold2)
            else:
                if debug:
                    print("Obstacle ahead but not too close — full linear allowed")
                goalLinearVel = self.maxLinearVel
    
            # Choose turn direction based on side readings
            if self.turnDir == 0:
                # Initialize based on which side has more space
                self.turnDir = 1 if leftReading > rightReading else -1
                if debug:
                    print(f"turnDir was 0, setting to {self.turnDir} based on space")
            
            # Angular velocity logic
            if leftReading < self.sideObsThreshold and centerReading < self.obsThreshold2 and rightReading < self.sideObsThreshold:
                if debug:
                    print("Both sides blocked → turning with previous turnDir")
                goalAngularVel = self.turnDir * self.maxAngularVel
            if leftReading < self.sideObsThreshold and centerReading < self.obsThreshold2 and rightReading < self.sideObsThreshold:
                if debug:
                    print("Both sides blocked → turning with previous turnDir")
                goalAngularVel = self.turnDir * self.maxAngularVel
    
            elif abs(leftReading - rightReading) / centerReading < 0.25:
                if debug:
                    print("Small left/right diff → maintain current angular direction")
                goalAngularVel = np.sign(goalAngularVel) * self.maxAngularVel
    
            elif abs(leftReading - rightReading) / centerReading > 0.125:
                if debug:
                    print("Large left/right difference → choose turnDir direction")
                goalAngularVel = self.turnDir * self.maxAngularVel
    
            elif leftReading > rightReading:
                if debug:
                    print("More space on left → turn left")
                goalAngularVel = self.maxAngularVel + min(self.obsThreshold2 / rightReading, 1)
    
            else:
                if debug:
                    print("More space on right → turn right")
                goalAngularVel = -self.maxAngularVel * min(self.obsThreshold2 / leftReading, 1)
    
            self.obstacle = True
    
        # ====== PURPLE BOX (Side obstacle while turning toward it) ======
        elif (rightReading < self.sideObsThreshold and goalAngle < 0) or \
             (leftReading < self.sideObsThreshold and goalAngle > 0):
            
            if debug:
                print("STATE: PURPLE BOX — SIDE OBSTACLE")
                print("Condition:",
                      f"Right<{self.sideObsThreshold} & goalAngle<0   OR   Left<{self.sideObsThreshold} & goalAngle>0")
            
            # Start with a modest forward speed
            goalLinearVel = 0.5 * self.maxLinearVel
            goalAngularVel = np.sign(goalAngle) * min(abs(goalAngle), self.maxAngularVel)
            if debug:
                print(f"Initial angular vel from goalAngle = {goalAngularVel:.3f}")
    
            if rightReading < self.sideObsThreshold and goalAngle < 0:
                if debug:
                    print("Right side blocked while turning right → zero angular vel")
                goalAngularVel = +self.maxAngularVel
            if leftReading < self.sideObsThreshold and goalAngle > 0:
                if debug:
                    print("Left side blocked while turning left → zero angular vel")
                goalAngularVel = -self.maxAngularVel
    
            # ====== YELLOW BOX (Obstacle detected Caution) ======
        elif (leftReading < self.sideObsThreshold or
              centerReading < self.obsThreshold1 or
              rightReading < self.sideObsThreshold):
            if debug:
                print("STATE: YELLOW BOX — GENERAL OBSTACLE")
            self.obstacle = True
        
            # If we don't already have a preferred turn direction, choose the more open side
            if self.turnDir == 0:
                self.turnDir = 1 if leftReading > rightReading else -1
                if debug:
                    print(f"YELLOW: setting turnDir={self.turnDir} based on space")
        
            # Back up slowly and turn away from the obstacle
            backup_speed = -0.4 * self.maxLinearVel      # reverse at 40% speed
            turn_speed   = 0.5 * self.maxAngularVel * self.turnDir  # gentle turn
        
            goalLinearVel  = backup_speed
            goalAngularVel = turn_speed
    
            # ====== GREEN BOX (Clear path) ======
        else:
            if debug:
                print("STATE: GREEN BOX — CLEAR PATH")
    
            if self.obstacle:
                if debug:
                    print("Previously in obstacle state → flipping turnDir")
                self.turnDir = -self.turnDir
                self.obstacle = False
    
            # Linear velocity toward goal
            if goalDist > self.approachThreshold:
                if debug:
                    print("Far from target — full linear velocity scaling")
                goalLinearVel = self.maxLinearVel * (2 * abs(1 - abs(goalAngle)) / math.pi)
            else:
                if debug:
                    print("Near target — reducing speed proportionally")
                goalLinearVel = self.maxLinearVel * (2 * abs(1 - abs(goalAngle)) / math.pi) * (goalDist / self.approachThreshold)
    
            # In GREEN BOX section:
            ANGLE_DEAD_ZONE = 0.1  # ~5.7 degrees
            
            # If goal is mostly behind, force a preferred direction
            if abs(goalAngle) > math.pi * 0.9 or self.fullSpin:
                print("OBJECT BEHIND CHOOSING LEFT TURN")
                if abs(goalAngle) < ANGLE_DEAD_ZONE:
                    self.fullSpin = False
                else:
                    self.fullSpin = True
                    # always rotate left in place when it's basically behind
                    goalAngle = math.pi * 0.9 
                    goalLinearVel = 0.0
                
            if abs(goalAngle) < ANGLE_DEAD_ZONE: # if angle is pointing towards goal
                goalAngularVel = 0.0
            else:
                goalAngularVel = np.sign(goalAngle) * min(abs(goalAngle), self.maxAngularVel)
                
            if debug:
                print(f"Angular vel toward goal = {goalAngularVel:.3f}") 
        
        # Convert (v,omega) to differential drive
        scale = 1.0 - (abs(goalAngle) / math.pi)
        scale = max(0.0, min(1.0, scale))   # clamp to [0,1]
        goalLinearVel *= scale
        leftVel  = goalLinearVel - 0.5 * goalAngularVel
        rightVel = goalLinearVel + 0.5 * goalAngularVel

        if debug:
            print(f"FINAL: goalLinearVel={goalLinearVel:.3f}, goalAngularVel={goalAngularVel:.3f}")
            print(f"Wheel speeds → Left={leftVel:.3f}, Right={rightVel:.3f}")
            print("-----------------------------------")
    
        return leftVel, rightVel