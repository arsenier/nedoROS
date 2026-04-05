from manim import *

class serial(Scene):
    def construct(self):
        self.camera.background_color = "#1a1a2e"

        # Актёры
        actors = VGroup(
            self.create_actor("Rpi", ""),
            self.create_actor("Arduino", ""),
        )
        actors.arrange(RIGHT, buff=7.0)
        actors.to_edge(UP, buff=0.8)

        # Линии жизни
        lifelines = VGroup(*[
            DashedLine(
                actor.get_bottom() + DOWN * 0.3,
                actor.get_bottom() + DOWN * 6.8,
                color=WHITE,
                dash_length=0.15
            )
            for actor in actors
        ])

        self.play(
            *[FadeIn(a) for a in actors],
            *[Create(l) for l in lifelines],
            run_time=1.5
        )

        # Функция пакета (сжатая рамка, уменьшенный шрифт)
        def packet(fields, width):
            box = RoundedRectangle(
                width=width,
                height=1.2,  # уменьшена высота
                corner_radius=0.25,
                color="#f5a623",
                stroke_width=3
            )
            box.set_fill("#0f1b2e", opacity=0.95)

            n = len(fields)
            cell_width = width / n

            separators = VGroup()
            texts = VGroup()

            for i in range(n):
                x_pos = box.get_left()[0] + cell_width * (i + 0.5)

                name = Text(fields[i][0], font_size=14, color="#00ffcc")   # уменьшен шрифт
                dtype = Text(fields[i][1], font_size=10, color="#ff6b6b")  # уменьшен шрифт

                txt = VGroup(name, dtype).arrange(DOWN, buff=0.04)
                txt.move_to([x_pos, box.get_center()[1], 0])

                texts.add(txt)

                if i < n - 1:
                    sep = Line(
                        start=[box.get_left()[0] + cell_width * (i + 1), box.get_bottom()[1] + 0.1, 0],
                        end=[box.get_left()[0] + cell_width * (i + 1), box.get_top()[1] - 0.1, 0],
                        color="#f5a623",
                        stroke_width=2
                    )
                    separators.add(sep)

            return VGroup(box, separators, texts)

        # Пакеты (сжаты только рамки, названия полей полные)
        top_packet = packet([
            ("0x01", "byte"),
            ("left_speed", "float"),
            ("right_speed", "float"),
            ("gripper", "byte"),
            ("checksum", "byte"),
        ], width=6.5)  # уменьшена ширина

        bottom_packet = packet([
            ("0x01", "byte"),
            ("x", "float"),
            ("y", "float"),
            ("theta", "float"),
            ("usik_l", "byte"),
            ("usik_r", "byte"),
            ("checksum", "byte"),
        ], width=7.8)  # уменьшена ширина

        top_packet.next_to(actors, DOWN, buff=1.1)
        bottom_packet.next_to(top_packet, DOWN, buff=0.7)

        # Стрелки
        arrow_top = Arrow(      # от Rpi → Arduino
            start=lifelines[1].get_center() + DOWN * 1.2,
            end=lifelines[0].get_center() + DOWN * 1.2,
            color="#00ffcc",
            stroke_width=4,
            buff=0.5
        )


        # Вторая стрелка от Arduino к Rpi НАД первым пакетом
        arrow_top_second = Arrow(
            start=lifelines[0].get_center() - DOWN * 2.9,
            end=lifelines[1].get_center() - DOWN * 2.9,
            color="#ff6b6b",
            stroke_width=4,
            buff=0.5
        )

        # Анимация
        self.play(Create(top_packet))
        self.play(Create(arrow_top))
        self.play(Create(arrow_top_second))  # новая стрелка сверху

        self.play(Create(bottom_packet))

        self.wait(2)

    def create_actor(self, name, role):
        box = RoundedRectangle(
            width=3.2,
            height=1.3,
            corner_radius=0.4,
            color="#e94560",
            fill_opacity=1,
            stroke_width=4
        )

        text = Text(name, font_size=30, color=BLACK, weight=BOLD)
        text.move_to(box.get_center())

        return VGroup(box, text)