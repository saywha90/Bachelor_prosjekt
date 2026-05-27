"""Regression tests for red/orange ball detection under glare."""

from __future__ import annotations

import cv2
import numpy as np

from vision.detector import BallColor, SimpleBallDetector


def _hsv_bgr(hue: int, sat: int, val: int) -> tuple[int, int, int]:
    hsv = np.uint8([[[hue, sat, val]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def test_red_orange_ball_with_glare_is_detected_as_one_red_ball():
    frame = np.full((240, 320, 3), (92, 92, 92), dtype=np.uint8)
    center = (170, 115)

    # Main body is orange-shifted red (H=32), outside the strict H<=25 red mask.
    cv2.circle(frame, center, 34, _hsv_bgr(32, 175, 230), -1, lineType=cv2.LINE_AA)

    # Small saturated red regions provide the seed pixels seen intermittently in
    # real calibration frames.
    cv2.circle(frame, (150, 103), 12, _hsv_bgr(6, 220, 220), -1, lineType=cv2.LINE_AA)
    cv2.circle(frame, (185, 130), 10, _hsv_bgr(178, 210, 215), -1, lineType=cv2.LINE_AA)

    # Broad desaturated highlight crossing the ball.  This used to split the red
    # mask into small, frame-sensitive fragments.
    cv2.ellipse(frame, (173, 112), (23, 9), -25, 0, 360, _hsv_bgr(24, 35, 248), -1, lineType=cv2.LINE_AA)

    detector = SimpleBallDetector(
        min_radius=18,
        max_radius=60,
        confidence_threshold=0.35,
        enable_adaptive_lighting=False,
        max_balls_per_color=10,
    )

    balls, _stats = detector.detect_balls(frame)
    red_balls = [ball for ball in balls if ball.color == BallColor.RED]

    assert len(red_balls) == 1
    ball = red_balls[0]
    assert abs(ball.center[0] - center[0]) <= 8
    assert abs(ball.center[1] - center[1]) <= 8
    assert 24 <= ball.radius <= 44


def test_standalone_orange_shifted_ball_without_strict_red_seed_is_detected():
    frame = np.full((240, 320, 3), (92, 92, 92), dtype=np.uint8)

    # Real red/orange balls can be shifted fully into H≈32 by glare/lighting,
    # leaving no strict-red seed pixels.  The detector should still accept the
    # component when it is a clean ball shape.
    center = (160, 120)
    cv2.circle(frame, center, 34, _hsv_bgr(32, 175, 230), -1, lineType=cv2.LINE_AA)

    detector = SimpleBallDetector(
        min_radius=18,
        max_radius=60,
        confidence_threshold=0.35,
        enable_adaptive_lighting=False,
        max_balls_per_color=10,
    )

    balls, _stats = detector.detect_balls(frame)
    red_balls = [ball for ball in balls if ball.color == BallColor.RED]

    assert len(red_balls) == 1
    assert abs(red_balls[0].center[0] - center[0]) <= 8
    assert abs(red_balls[0].center[1] - center[1]) <= 8


def test_separate_irregular_orange_background_without_red_seed_is_not_promoted_to_red_ball():
    frame = np.full((240, 320, 3), (92, 92, 92), dtype=np.uint8)

    # Same orange hue as the glare-shifted body above, but an irregular patch
    # instead of a ball.  Standalone orange is accepted only when ball-shaped.
    pts = np.array([[110, 95], [178, 87], [204, 117], [187, 154], [132, 143]], dtype=np.int32)
    cv2.fillPoly(frame, [pts], _hsv_bgr(32, 175, 230), lineType=cv2.LINE_AA)

    detector = SimpleBallDetector(
        min_radius=18,
        max_radius=60,
        confidence_threshold=0.35,
        enable_adaptive_lighting=False,
        max_balls_per_color=10,
    )

    balls, _stats = detector.detect_balls(frame)

    assert [ball for ball in balls if ball.color == BallColor.RED] == []


def test_elongated_red_robot_arm_like_blob_is_rejected_as_non_round():
    frame = np.full((240, 320, 3), (92, 92, 92), dtype=np.uint8)

    # Saturated red like robot plastic/tape, but deliberately long/oval instead
    # of ball-shaped.  It must not become a red ball through HSV or Hough.
    cv2.ellipse(frame, (160, 120), (58, 22), 8, 0, 360, _hsv_bgr(4, 220, 225), -1, lineType=cv2.LINE_AA)

    detector = SimpleBallDetector(
        min_radius=18,
        max_radius=80,
        confidence_threshold=0.35,
        enable_adaptive_lighting=False,
        max_balls_per_color=10,
    )

    balls, _stats = detector.detect_balls(frame)

    assert [ball for ball in balls if ball.color == BallColor.RED] == []


def test_rounded_red_wrapper_or_tape_strip_is_rejected_as_non_round():
    frame = np.full((240, 320, 3), (92, 92, 92), dtype=np.uint8)
    color = _hsv_bgr(6, 210, 220)

    # A capsule/rounded rectangle has high solidity and no sharp corners, which
    # used to be close to the old gates, but it fills too little of its enclosing
    # circle and is not a round object.
    cv2.rectangle(frame, (112, 96), (208, 144), color, -1, lineType=cv2.LINE_AA)
    cv2.circle(frame, (112, 120), 24, color, -1, lineType=cv2.LINE_AA)
    cv2.circle(frame, (208, 120), 24, color, -1, lineType=cv2.LINE_AA)

    detector = SimpleBallDetector(
        min_radius=18,
        max_radius=80,
        confidence_threshold=0.35,
        enable_adaptive_lighting=False,
        max_balls_per_color=10,
    )

    balls, _stats = detector.detect_balls(frame)

    assert [ball for ball in balls if ball.color == BallColor.RED] == []


def test_elongated_blue_jeans_like_patch_is_rejected_as_non_round():
    frame = np.full((240, 320, 3), (92, 92, 92), dtype=np.uint8)

    # Denim-like muted blue: inside the blue hue range, but elongated and thus
    # not ball-shaped.
    cv2.ellipse(frame, (160, 120), (64, 26), -18, 0, 360, _hsv_bgr(106, 95, 130), -1, lineType=cv2.LINE_AA)

    detector = SimpleBallDetector(
        min_radius=18,
        max_radius=80,
        confidence_threshold=0.35,
        enable_adaptive_lighting=False,
        max_balls_per_color=10,
    )

    balls, _stats = detector.detect_balls(frame)

    assert [ball for ball in balls if ball.color == BallColor.BLUE] == []


def test_low_light_cyan_blue_ball_is_detected_under_dim_manual_exposure():
    frame = np.full((240, 320, 3), (18, 18, 18), dtype=np.uint8)
    center = (160, 120)

    # Dim manual OAK exposure can shift the blue calibration ball toward cyan and
    # keep value/saturation below the previous light-blue HSV floor.
    cv2.circle(frame, center, 34, _hsv_bgr(82, 58, 62), -1, lineType=cv2.LINE_AA)
    cv2.ellipse(frame, (148, 108), (17, 8), -25, 0, 360, _hsv_bgr(82, 24, 125), -1, lineType=cv2.LINE_AA)
    cv2.ellipse(frame, (171, 132), (30, 17), -20, 0, 360, _hsv_bgr(86, 46, 38), -1, lineType=cv2.LINE_AA)

    detector = SimpleBallDetector(
        min_radius=18,
        max_radius=80,
        confidence_threshold=0.35,
        enable_adaptive_lighting=False,
        max_balls_per_color=10,
    )

    balls, _stats = detector.detect_balls(frame)
    blue_balls = [ball for ball in balls if ball.color == BallColor.BLUE]

    assert len(blue_balls) == 1
    assert abs(blue_balls[0].center[0] - center[0]) <= 8
    assert abs(blue_balls[0].center[1] - center[1]) <= 8
    assert 24 <= blue_balls[0].radius <= 44


def test_low_light_blue_ball_with_shadowed_lower_half_still_counts_as_round_ball():
    frame = np.full((240, 320, 3), (18, 18, 18), dtype=np.uint8)
    center = (160, 120)

    # The visible object is physically round, but the lower half is so dark and
    # low-saturation that the previous strict mask kept only a top crescent.  That
    # crescent failed circularity/fill and the detector concluded "not a ball".
    cv2.circle(frame, center, 34, _hsv_bgr(88, 125, 100), -1, lineType=cv2.LINE_AA)
    cv2.ellipse(frame, (166, 132), (32, 20), 0, 0, 360, _hsv_bgr(88, 32, 26), -1, lineType=cv2.LINE_AA)
    cv2.ellipse(frame, (149, 107), (16, 9), -30, 0, 360, _hsv_bgr(88, 28, 160), -1, lineType=cv2.LINE_AA)

    detector = SimpleBallDetector(
        min_radius=18,
        max_radius=80,
        confidence_threshold=0.35,
        enable_adaptive_lighting=False,
        max_balls_per_color=10,
    )

    balls, _stats = detector.detect_balls(frame)
    blue_balls = [ball for ball in balls if ball.color == BallColor.BLUE]

    assert len(blue_balls) == 1
    assert abs(blue_balls[0].center[0] - center[0]) <= 8
    assert abs(blue_balls[0].center[1] - center[1]) <= 8
    assert 24 <= blue_balls[0].radius <= 44


def test_low_light_cyan_blue_jeans_like_patch_still_rejected_as_non_round():
    frame = np.full((240, 320, 3), (18, 18, 18), dtype=np.uint8)

    # Same recovered low-light cyan/blue colour family as the valid ball above,
    # but elongated like denim/arm fabric, so shape gates must reject it.
    cv2.ellipse(frame, (160, 120), (66, 26), 15, 0, 360, _hsv_bgr(82, 58, 62), -1, lineType=cv2.LINE_AA)

    detector = SimpleBallDetector(
        min_radius=18,
        max_radius=80,
        confidence_threshold=0.35,
        enable_adaptive_lighting=False,
        max_balls_per_color=10,
    )

    balls, _stats = detector.detect_balls(frame)

    assert [ball for ball in balls if ball.color == BallColor.BLUE] == []


def test_sunlit_blue_ball_not_hidden_by_low_saturation_blueish_floor_glare():
    frame = np.full((260, 380, 3), _hsv_bgr(104, 24, 128), dtype=np.uint8)
    center = (260, 130)

    # Direct sun / auto white balance can make the grey floor look weakly blue.
    # This must not be repaired into a huge blue object that hides the real ball.
    cv2.rectangle(frame, (0, 0), (380, 70), _hsv_bgr(103, 32, 170), -1, lineType=cv2.LINE_AA)
    cv2.ellipse(frame, (115, 205), (95, 36), -14, 0, 360, _hsv_bgr(108, 34, 185), -1, lineType=cv2.LINE_AA)
    cv2.ellipse(frame, (330, 40), (65, 20), 8, 0, 360, _hsv_bgr(100, 30, 210), -1, lineType=cv2.LINE_AA)

    # Real blue ball: saturated seed plus a bright low-saturation highlight.
    cv2.circle(frame, center, 32, _hsv_bgr(104, 165, 190), -1, lineType=cv2.LINE_AA)
    cv2.ellipse(frame, (252, 119), (18, 9), -20, 0, 360, _hsv_bgr(104, 28, 250), -1, lineType=cv2.LINE_AA)
    cv2.ellipse(frame, (268, 143), (27, 15), 0, 0, 360, _hsv_bgr(108, 95, 125), -1, lineType=cv2.LINE_AA)

    detector = SimpleBallDetector(
        min_radius=18,
        max_radius=80,
        confidence_threshold=0.35,
        enable_adaptive_lighting=False,
        max_balls_per_color=1,
    )

    balls, _stats = detector.detect_balls(frame)
    blue_balls = [ball for ball in balls if ball.color == BallColor.BLUE]

    assert len(blue_balls) == 1
    assert abs(blue_balls[0].center[0] - center[0]) <= 10
    assert abs(blue_balls[0].center[1] - center[1]) <= 10
    assert 24 <= blue_balls[0].radius <= 46


def test_circular_red_and_blue_balls_still_pass_round_object_gates():
    frame = np.full((260, 380, 3), (92, 92, 92), dtype=np.uint8)
    red_center = (120, 130)
    blue_center = (260, 130)

    cv2.circle(frame, red_center, 32, _hsv_bgr(5, 215, 225), -1, lineType=cv2.LINE_AA)
    cv2.circle(frame, blue_center, 32, _hsv_bgr(108, 165, 175), -1, lineType=cv2.LINE_AA)

    detector = SimpleBallDetector(
        min_radius=18,
        max_radius=70,
        confidence_threshold=0.35,
        enable_adaptive_lighting=False,
        max_balls_per_color=10,
    )

    balls, _stats = detector.detect_balls(frame)
    red_balls = [ball for ball in balls if ball.color == BallColor.RED]
    blue_balls = [ball for ball in balls if ball.color == BallColor.BLUE]

    assert len(red_balls) == 1
    assert len(blue_balls) == 1
    assert abs(red_balls[0].center[0] - red_center[0]) <= 8
    assert abs(red_balls[0].center[1] - red_center[1]) <= 8
    assert abs(blue_balls[0].center[0] - blue_center[0]) <= 8
    assert abs(blue_balls[0].center[1] - blue_center[1]) <= 8
