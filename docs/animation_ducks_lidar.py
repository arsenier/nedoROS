from manim import *
import math

class object_docking(Scene):
    def construct(self):
        self.camera.background_color = "#1a1a2e"
        
        robot_body = Circle(radius=0.3, color="#e94560")
        robot_body.set_fill(opacity=0.5, color="#e94560")
        
        left_claw = ArcBetweenPoints(
            start=[0.3, 0, 0],
            end=[0.65, -0.15, 0],
            angle=0.6,
            color="#f5a623",
            stroke_width=5
        )
        
        right_claw = ArcBetweenPoints(
            start=[0.3, 0, 0],
            end=[0.65, 0.15, 0],
            angle=-0.6,
            color="#f5a623",
            stroke_width=5
        )
        
        robot = VGroup(robot_body, left_claw, right_claw)
        
        robot_position = np.array([-0.2, -0.5, 0])
        robot.move_to(robot_position)
        robot.rotate(-PI/7)
        
        self.play(LaggedStart(
            Create(robot_body), Create(left_claw), 
            Create(right_claw), lag_ratio=0.2), run_time=2)
        self.wait(0.5)
        
        target_point = Dot(color="#00ffcc", radius=0.08)
        target_glow = Circle(radius=0.2, color="#00ffcc", stroke_width=2)
        target_glow.set_fill(opacity=0.3, color="#00ffcc")
        target = VGroup(target_point)
        target_position = np.array([0.5, 0.3, 0])
        target.move_to(target_position)
        
        self.play(FadeIn(target_point, scale=0.5))
        self.wait(0.5)
        
        data_panel = RoundedRectangle(width=5, height=2.5, corner_radius=0.2, color="#f5a623", stroke_width=2, fill_opacity=0.2)
        data_panel.to_corner(UP + LEFT, buff=0.5)
        
        dist_display = Text("Расстояние: 0.00", font="GOST Type AU", color="#00ffcc", font_size=20)
        dist_display.move_to(data_panel.get_center() + np.array([-1, 0.3, 0]))
        
        reg_display = Text("Разница угла: 0.00", font="GOST Type AU", color="#ffd93d", font_size=20)
        reg_display.move_to(data_panel.get_center() + np.array([-1, -0.3, 0]))
        
        self.play(Create(data_panel), Write(dist_display), Write(reg_display))
        self.wait(1)
        
        robot_pose = {"x": -0.2, "y": -0.5, "theta": -PI / 7}
        
        distotcl = 0.1
        k_forward = 0.4
        k_turn = 0.4
        maxline = 0.2
        maxz = 1.0
        robot_gotopoint_dist_threshold = 0.1
        
        const_dist_gripper = 0.25
        porog_turn = 20.0 * math.pi / 180
        porog_dist = 0.1
        dock_timer = 0.0
        dock_time_threshold = 1.0
        timer_period = 0.1
        gripper_closed = False
        
        trail = VGroup()
        self.add(trail)
        
        cmr_line = None
        cma_arc = None
        
        def update_robot():
            nonlocal dist_display, reg_display, dock_timer, gripper_closed, cmr_line, cma_arc
            
            while True:
                dx = target_position[0] - robot_pose["x"]
                dy = target_position[1] - robot_pose["y"]
                distance = math.sqrt(dx*dx + dy*dy)
                
                desired_angle = math.atan2(dy, dx)
                angle_error = desired_angle - robot_pose["theta"]
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
                
                if cmr_line:
                    self.remove(cmr_line)
                if cma_arc:
                    self.remove(cma_arc)
                
                robot_center = np.array([robot_pose["x"], robot_pose["y"], 0])
                cmr_line = Line(robot_center, target_position, color="#00ffcc", stroke_width=3)
                
                robot_direction = np.array([math.cos(robot_pose["theta"]), math.sin(robot_pose["theta"]), 0])
                target_direction = np.array([dx/distance, dy/distance, 0]) if distance > 0 else robot_direction
                
                angle_start = robot_pose["theta"]
                angle_end = desired_angle
                
                arc_radius = 0.5
                if angle_error > 0:
                    cma_arc = ArcBetweenPoints(
                        robot_center + robot_direction * arc_radius,
                        robot_center + target_direction * arc_radius,
                        angle=angle_error,
                        color="#ffd93d",
                        stroke_width=4
                    )
                else:
                    cma_arc = ArcBetweenPoints(
                        robot_center + robot_direction * arc_radius,
                        robot_center + target_direction * arc_radius,
                        angle=angle_error,
                        color="#ffd93d",
                        stroke_width=4
                    )
                
                self.add(cmr_line, cma_arc)
                
                ddist = distance - const_dist_gripper
                
                if abs(angle_error) > porog_turn:
                    v = 0.0
                    anglereg = k_turn * angle_error
                else:
                    v = k_forward * ddist
                    anglereg = k_turn * angle_error
                
                v = min(max(v, -maxline), maxline)
                anglereg = min(max(anglereg, -maxz), maxz)
                
                if abs(angle_error) < (porog_turn / 2) and abs(ddist) < porog_dist:
                    dock_timer += timer_period
                else:
                    dock_timer = 0.0
                
                if distance < 0.1:
                    v = 0.0
                
                new_x = robot_pose["x"] + v * math.cos(robot_pose["theta"]) * timer_period
                new_y = robot_pose["y"] + v * math.sin(robot_pose["theta"]) * timer_period
                new_theta = robot_pose["theta"] + anglereg * timer_period
                
                robot_pose["x"] = new_x
                robot_pose["y"] = new_y
                robot_pose["theta"] = new_theta
                
                robot.move_to(np.array([new_x, new_y, 0]))
                robot.rotate(anglereg * timer_period, about_point=robot.get_center())
                
                trail.add(Dot(point=[new_x, new_y, 0], radius=0.03, color="#e94560", fill_opacity=0.3))
                
                new_dist = Text(f"Расстояние (cmr): {max(0, distance / 3.1):.2f} м", font="GOST Type AU", color="#00ffcc", font_size=20)
                new_dist.move_to(dist_display.get_center())
                self.remove(dist_display)
                self.add(new_dist)
                dist_display = new_dist
                
                new_reg = Text(f"Разница угла (cma): {angle_error:.2f} рад", font="GOST Type AU", color="#ffd93d", font_size=20)
                new_reg.move_to(reg_display.get_center())
                self.remove(reg_display)
                self.add(new_reg)
                reg_display = new_reg
                
                if dock_timer > dock_time_threshold and not gripper_closed:
                    gripper_closed = True
                    self.wait(0.5)
                    break
                
                if distance <= robot_gotopoint_dist_threshold:
                    break
                
                self.wait(timer_period)
        
        update_robot()
        
        self.wait(2)