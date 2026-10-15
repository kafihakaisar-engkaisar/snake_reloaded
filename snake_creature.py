import pygame
import math
import numpy as np

pygame.init()

# Window Setup
WIDTH, HEIGHT = 1100, 750
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Procedural Skeletal IK Creature")
clock = pygame.time.Clock()

# Colors
BG_COLOR = (12, 5, 32)         # Deep purple-black
GLOW_COLOR = (80, 140, 255, 35) # Soft neon blue glow
BONE_COLOR = (240, 245, 255)    # Bright skeletal white
JOINT_COLOR = (200, 220, 255)

# --- 2-Bone Analytical Inverse Kinematics ---
def solve_ik_2bone(origin, target, l1, l2, flip=False):
    """Calculates elbow/knee joint position using law of cosines."""
    ox, oy = origin
    tx, ty = target
    dx, dy = tx - ox, ty - oy
    dist = math.hypot(dx, dy)
    
    # Clamp distance so the limb doesn't break if target is out of reach
    dist = max(1e-4, min(l1 + l2 - 1e-4, dist))
    base_angle = math.atan2(dy, dx)
    
    # Law of cosines
    cos_val = (l1**2 + dist**2 - l2**2) / (2 * l1 * dist)
    cos_val = max(-1.0, min(1.0, cos_val))
    interior_angle = math.acos(cos_val)
    
    if flip:
        joint_angle = base_angle - interior_angle
    else:
        joint_angle = base_angle + interior_angle
        
    jx = ox + math.cos(joint_angle) * l1
    jy = oy + math.sin(joint_angle) * l1
    return (jx, jy)

# --- Procedural Leg Class ---
class ProceduralLeg:
    def __init__(self, segment_idx, side, leg_len1=65, leg_len2=65):
        self.seg_idx = segment_idx
        self.side = side  # -1 for left, 1 for right
        self.l1 = leg_len1
        self.l2 = leg_len2
        
        self.foot_pos = [0, 0]
        self.step_target = [0, 0]
        self.is_stepping = False
        self.step_progress = 1.0
        self.step_start = [0, 0]
        self.max_reach = (self.l1 + self.l2) * 0.85

    def update(self, origin, body_angle):
        # Calculate preferred foot placement in the direction of movement
        perp_angle = body_angle + (math.pi / 2 * self.side)
        rest_dist = 95
        forward_lead = 45
        
        ideal_x = origin[0] + math.cos(perp_angle) * rest_dist + math.cos(body_angle) * forward_lead
        ideal_y = origin[1] + math.sin(perp_angle) * rest_dist + math.sin(body_angle) * forward_lead
        
        # Check if foot is too far from its body anchor
        dist_to_anchor = math.hypot(self.foot_pos[0] - origin[0], self.foot_pos[1] - origin[1])
        if dist_to_anchor > self.max_reach and not self.is_stepping:
            self.is_stepping = True
            self.step_progress = 0.0
            self.step_start = list(self.foot_pos)
            self.step_target = [ideal_x, ideal_y]
            
        # Animate stepping
        if self.is_stepping:
            self.step_progress += 0.16  # Step speed
            t = min(1.0, self.step_progress)
            # Smooth interpolation
            smooth_t = (1 - math.cos(t * math.pi)) / 2
            self.foot_pos[0] = self.step_start[0] + (self.step_target[0] - self.step_start[0]) * smooth_t
            self.foot_pos[1] = self.step_start[1] + (self.step_target[1] - self.step_start[1]) * smooth_t
            if self.step_progress >= 1.0:
                self.is_stepping = False

    def draw(self, surface, origin):
        knee = solve_ik_2bone(origin, self.foot_pos, self.l1, self.l2, flip=(self.side < 0))
        # Draw limb segments
        pygame.draw.line(surface, BONE_COLOR, origin, knee, 2)
        pygame.draw.line(surface, BONE_COLOR, knee, self.foot_pos, 2)
        pygame.draw.circle(surface, JOINT_COLOR, (int(knee[0]), int(knee[1])), 3)
        pygame.draw.circle(surface, BONE_COLOR, (int(self.foot_pos[0]), int(self.foot_pos[1])), 2)

# --- Procedural Spine / Creature Class ---
class SkeletalCreature:
    def __init__(self, num_segments=52, segment_dist=12):
        self.num_segments = num_segments
        self.seg_dist = segment_dist
        
        # Positions [N, 2]
        self.points = np.zeros((num_segments, 2), dtype=float)
        self.points[:, 0] = WIDTH // 2
        for i in range(num_segments):
            self.points[i, 1] = HEIGHT // 2 + i * segment_dist

        # Attach procedural legs along mid-body segments
        self.legs = []
        leg_attachments = [ 12, 20,28, 36,44,52]  # Attach legs to these vertebrae
        for idx in leg_attachments:
            # Left leg
            leg_l = ProceduralLeg(idx, -1)
            leg_l.foot_pos = [self.points[idx, 0] - 50, self.points[idx, 1]]
            # Right leg
            leg_r = ProceduralLeg(idx, 1)
            leg_r.foot_pos = [self.points[idx, 0] + 50, self.points[idx, 1]]
            self.legs.extend([leg_l, leg_r])

    def update(self, target_pos):
        # 1. Head smoothly pursues target
        head = self.points[0]
        dx = target_pos[0] - head[0]
        dy = target_pos[1] - head[1]
        dist = math.hypot(dx, dy)
        
        if dist > 2:
            speed = min(dist * 0.15, 9.0)
            head[0] += (dx / dist) * speed
            head[1] += (dy / dist) * speed

        # 2. Forward-backward distance constraints (Chain IK)
        for i in range(1, self.num_segments):
            prev = self.points[i - 1]
            curr = self.points[i]
            
            p_dx = curr[0] - prev[0]
            p_dy = curr[1] - prev[1]
            seg_len = math.hypot(p_dx, p_dy)
            
            if seg_len > 0:
                scale = self.seg_dist / seg_len
                curr[0] = prev[0] + p_dx * scale
                curr[1] = prev[1] + p_dy * scale

        # 3. Update Legs
        for leg in self.legs:
            idx = leg.seg_idx
            origin = (self.points[idx, 0], self.points[idx, 1])
            # Angle of segment
            dir_x = self.points[idx - 1, 0] - self.points[idx, 0]
            dir_y = self.points[idx - 1, 1] - self.points[idx, 1]
            body_angle = math.atan2(dir_y, dir_x)
            leg.update(origin, body_angle)

    def draw(self, surface):
        glow_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        
        # 1. Draw Legs
        for leg in self.legs:
            idx = leg.seg_idx
            origin = (self.points[idx, 0], self.points[idx, 1])
            leg.draw(surface, origin)

        # 2. Draw Spine & Perpendicular Ribs
        for i in range(self.num_segments):
            pos = self.points[i]
            
            # Direction vector
            if i < self.num_segments - 1:
                dx = self.points[i + 1, 0] - pos[0]
                dy = self.points[i + 1, 1] - pos[1]
            else:
                dx = pos[0] - self.points[i - 1, 0]
                dy = pos[1] - self.points[i - 1, 1]
                
            angle = math.atan2(dy, dx)
            perp_angle = angle + math.pi / 2

            # Dynamic rib width profile (egg/serpent curve)
            progress = i / self.num_segments
            if progress < 0.15:
                rib_w = math.sin((progress / 0.15) * (math.pi / 2)) * 26
            else:
                rib_w = math.cos(((progress - 0.15) / 0.85) * (math.pi / 2)) * 26

            # Compute rib tip endpoints
            left_rib = (pos[0] + math.cos(perp_angle) * rib_w,
                        pos[1] + math.sin(perp_angle) * rib_w)
            right_rib = (pos[0] - math.cos(perp_angle) * rib_w,
                         pos[1] - math.sin(perp_angle) * rib_w)

            # Rib glow & bones
            if rib_w > 2:
                pygame.draw.line(glow_surf, GLOW_COLOR, left_rib, right_rib, 5)
                pygame.draw.line(surface, BONE_COLOR, left_rib, right_rib, 1)

            # Vertebra center point
            pygame.draw.circle(surface, JOINT_COLOR, (int(pos[0]), int(pos[1])), 2)

        # 3. Draw Spine backbone cord
        points_list = [(int(p[0]), int(p[1])) for p in self.points]
        pygame.draw.lines(surface, BONE_COLOR, False, points_list, 2)

        # Composite glow layer
        surface.blit(glow_surf, (0, 0))


# --- Main Loop ---
creature = SkeletalCreature(num_segments=75, segment_dist=11)
running = True

while running:
    clock.tick(60)
    screen.fill(BG_COLOR)
    
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            running = False

    mouse_pos = pygame.mouse.get_pos()
    
    creature.update(mouse_pos)
    creature.draw(screen)

    pygame.display.flip()

pygame.quit()