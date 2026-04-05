from manim import *
import math

class RobotNavigation(Scene):
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
        
        robot_position = np.array([-5, -1.5, 0])
        robot.move_to(robot_position)
        robot.rotate(-PI/2)
        
        self.play(LaggedStart(
            Create(robot_body), Create(left_claw), 
            Create(right_claw), lag_ratio=0.2), run_time=2)
        self.wait(0.5)

        target_point = Dot(color="#00ffcc", radius=0.08)
        target_glow = Circle(radius=0.2, color="#00ffcc", stroke_width=2)
        target_glow.set_fill(opacity=0.3, color="#00ffcc")
        target = VGroup(target_point, target_glow)
        target_position = np.array([5, 2, 0])
        target.move_to(target_position)
        
        self.play(FadeIn(target_point, scale=0.5), Create(target_glow))
        self.wait(0.5)
        
        radius_circle = Circle(radius=0.5, color="#f5a623", stroke_width=2, stroke_opacity=0.5)
        radius_circle.move_to(target_position)
        self.play(Create(radius_circle))
        self.wait(0.5)
        
        data_panel = RoundedRectangle(width=5, height=2.5, corner_radius=0.2, color="#f5a623", stroke_width=2, fill_opacity=0.2)
        data_panel.to_corner(UP + LEFT, buff=0.5)
        
        dist_display = Text("Расстояние: 0.00", font="GOST Type AU", color="#00ffcc", font_size=20)
        dist_display.move_to(data_panel.get_center() + np.array([-1, 0.3, 0]))
        
        reg_display = Text("Разница угла: 0.00", font="GOST Type AU", color="#00ffcc", font_size=20)
        reg_display.move_to(data_panel.get_center() + np.array([-1, -0.3, 0]))
        
        self.play(Create(data_panel), Write(dist_display), Write(reg_display))
        self.wait(1)
        
        robot_pose = {"x": -5.0, "y": -1.5, "theta": -PI / 2}
        
        distotcl = 0.1
        k_forward = 0.4
        k_turn = 0.4
        maxline = 0.2
        maxz = 1.0
        robot_gotopoint_dist_threshold = 0.9
        
        trail = VGroup()
        self.add(trail)
        
        def update_robot():
            nonlocal dist_display, reg_display
            while True:
                dx = target_position[0] - robot_pose["x"]
                dy = target_position[1] - robot_pose["y"]
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance <= robot_gotopoint_dist_threshold:
                    break
                
                desired_angle = math.atan2(dy, dx)
                angle_error = desired_angle - robot_pose["theta"]
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
                
                if abs(distance) > distotcl:
                    v = min(distance * k_forward, maxline)
                    anglereg = angle_error * k_turn
                    if anglereg > maxz:
                        anglereg = maxz
                    elif anglereg < -maxz:
                        anglereg = -maxz
                else:
                    v = 0
                    anglereg = 0
                
                new_x = robot_pose["x"] + v * math.cos(robot_pose["theta"]) * 0.2
                new_y = robot_pose["y"] + v * math.sin(robot_pose["theta"]) * 0.2
                new_theta = robot_pose["theta"] + anglereg * 0.2
                
                robot_pose["x"] = new_x
                robot_pose["y"] = new_y
                robot_pose["theta"] = new_theta
                
                robot.move_to(np.array([new_x, new_y, 0]))
                robot.rotate(anglereg * 0.2, about_point=robot.get_center())
                
                trail.add(Dot(point=[new_x, new_y, 0], radius=0.02, color="#e94560", fill_opacity=0.3))
                
                new_dist = Text(f"Расстояние: {max(0, distance * 0.33):.2f}", font="GOST Type AU", color="#00ffcc", font_size=20)
                new_dist.move_to(dist_display.get_center())
                self.remove(dist_display)
                self.add(new_dist)
                dist_display = new_dist
                
                new_reg = Text(f"Разница угла: {angle_error:.2f}", font="GOST Type AU", color="#00ffcc", font_size=20)
                new_reg.move_to(reg_display.get_center())
                self.remove(reg_display)
                self.add(new_reg)
                reg_display = new_reg
                
                self.wait(0.1)
        
        update_robot()
        
        
        self.wait(2)