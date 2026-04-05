from manim import *
import numpy as np
import math

class behaviour_with_claws_and_spa(Scene):
    def construct(self):
        self.camera.background_color = "#1a1a2e"

        field = RoundedRectangle(width=6, height=5, corner_radius=0.3, color="#f5a623")
        field.set_fill("#16213e", opacity=0.4)
        field.to_edge(RIGHT, buff=0.5)

        base = Square(side_length=0.7, color="#f5a623")
        base.set_fill("#f5a623", opacity=0.6)
        base.move_to(field.get_corner(DR) + LEFT*0.5 + UP*0.5)

        ducks = VGroup()
        for col in range(2):
            for row in range(4):
                x = 0.5 + col * 0.6  # Сдвинуто левее (было 1.5)
                y = 1.2 - row * 0.9
                pos = field.get_center() + np.array([x, y, 0])
                duck = Dot(pos, color="#00ffcc", radius=0.07)
                ducks.add(duck)

        # Create robot with proper claw positioning (relative to robot body)
        robot_body = Circle(radius=0.18, color="#e94560")
        robot_body.set_fill("#e94560", opacity=0.8)
        
        left_claw = ArcBetweenPoints(
            start=[0.18, 0, 0],
            end=[0.45, -0.12, 0],
            angle=0.6,
            color="#f5a623",
            stroke_width=5
        )
        
        right_claw = ArcBetweenPoints(
            start=[0.18, 0, 0],
            end=[0.45, 0.12, 0],
            angle=-0.6,
            color="#f5a623",
            stroke_width=5
        )
        
        robot = VGroup(robot_body, left_claw, right_claw)
        # Робот стартует напротив позиции старта (левая сторона поля)
        robot.move_to(field.get_right() - RIGHT * 0.3)
        
        # Track robot rotation separately
        robot_angle = -PI/2  # Initial angle (facing right)
        robot.rotate(robot_angle)

        state_box = RoundedRectangle(width=4, height=1.2, corner_radius=0.2, color="#f5a623")
        state_box.set_fill("#16213e", opacity=0.7)
        state_box.to_edge(LEFT, buff=0.5)
        state_text = Text("WAIT_TARGET", font_size=26, color="#00ffcc").move_to(state_box.get_center())

        itspa_box = RoundedRectangle(width=4, height=1.8, corner_radius=0.2, color="#e94560")
        itspa_box.set_fill("#16213e", opacity=0.7)
        itspa_box.next_to(state_box, UP, buff=0.3)

        spa_labels = ["SENSE", "PLAN", "ACT"]
        spa_colors = {"SENSE": "#00ffcc", "PLAN": "#f5a623", "ACT": "#e94560"}

        spa_texts = VGroup(*[
            Text(label, font_size=18, color="#555555").move_to(
                itspa_box.get_center() + np.array([-1.2 + i*1.2, 0.4, 0])
            )
            for i, label in enumerate(spa_labels)
        ])

        self.play(Create(field), Create(base), FadeIn(ducks), Create(robot))
        self.play(Create(state_box), Write(state_text))
        self.play(Create(itspa_box))
        self.add(*spa_texts)
        self.wait(0.5)

        def set_state(name, spa, subs=[]):
            new_state = Text(name, font_size=26, color="#00ffcc").move_to(state_text.get_center())
            self.play(Transform(state_text, new_state), run_time=0.4)
            spa_list = spa.split(" + ")
            for s in spa_labels:
                if s in spa_list:
                    self.play(spa_texts[spa_labels.index(s)].animate.set_color(spa_colors[s]), run_time=0.2)
                else:
                    self.play(spa_texts[spa_labels.index(s)].animate.set_color("#555555"), run_time=0.2)
            
            # Clear previous sub_texts
            if hasattr(self, 'current_subs'):
                for st in self.current_subs:
                    self.remove(st)
            
            # Create new sub_texts inside the itspa_box
            self.current_subs = []
            for i, sub in enumerate(subs):
                sub_text = Text(f"{sub}", font_size=14, color="#aaaaaa")
                sub_text.move_to(itspa_box.get_center() + np.array([0, -0.3 - i*0.3, 0]))
                self.add(sub_text)
                self.current_subs.append(sub_text)
            
            self.wait(0.5)

        def go_to(target, carried_duck=None):
            nonlocal robot_angle
            pose = robot.get_center()
            
            for _ in range(50):  # Increased iterations for smoother movement
                dx = target[0] - pose[0]
                dy = target[1] - pose[1]
                dist = math.sqrt(dx*dx + dy*dy)
                if dist < 0.15:
                    break
                
                # Calculate desired angle
                desired_angle = math.atan2(dy, dx)
                angle_diff = desired_angle - robot_angle
                # Normalize angle difference to [-PI, PI]
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                
                # Movement
                step = min(0.2, dist * 0.3)
                new_pose = np.array([
                    pose[0] + step * math.cos(robot_angle),
                    pose[1] + step * math.sin(robot_angle),
                    0
                ])
                
                # Rotation
                rot_step = min(0.3, abs(angle_diff)) * (1 if angle_diff > 0 else -1)
                new_angle = robot_angle + rot_step
                
                # Apply transformations - двигаем только робота, утка остается на месте
                animations = [robot.animate.move_to(new_pose).rotate(rot_step, about_point=robot.get_center())]
                
                # Если утка уже схвачена, двигаем её вместе с роботом
                if carried_duck and hasattr(carried_duck, 'is_carried') and carried_duck.is_carried:
                    animations.append(carried_duck.animate.move_to(new_pose))
                
                self.play(
                    *animations,
                    run_time=0.05
                )
                
                pose = new_pose
                robot_angle = new_angle

        for i in range(len(ducks)):
            target_duck = ducks[i]

            set_state("WAIT_TARGET", "SENSE", subs=["/image_raw_charuco_objects", "/goal_pose"])
            set_state("GO_TO_TARGET", "PLAN + ACT", subs=["/cmd_vel", "/goal_pose"])
            
            # Робот едет к утке, утка остается на месте
            go_to(target_duck.get_center(), carried_duck=None)

            set_state("DOCK_WITH_DUCK", "SENSE + PLAN + ACT", subs=["/scan", "/cmd_vel"])
            self.play(target_duck.animate.set_color("#e94560"), run_time=0.3)

            set_state("GRAB_THE_DUCK", "ACT", subs=["/gripper"])
            self.play(robot.animate.scale(1.2), run_time=0.2)
            self.play(robot.animate.scale(1/1.2), run_time=0.2)
            
            # Помечаем утку как схваченную
            target_duck.is_carried = True

            set_state("GO_TO_PBASE", "PLAN + ACT", subs=["/cmd_vel", "/pfield_pose"])
            # Теперь утка движется вместе с роботом
            go_to(base.get_center(), carried_duck=target_duck)

            set_state("DROP_THE_DUCK", "ACT", subs=["/gripper"])
            self.play(target_duck.animate.move_to(base.get_center()), run_time=0.5)
            target_duck.is_carried = False

        set_state("WAIT_TIME", "SENSE", subs=["/time"])
        self.wait(2)