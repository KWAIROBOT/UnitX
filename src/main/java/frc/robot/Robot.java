// 2025 SSRC 3Team UNIT X
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DataLogManager; // 로그 매니저

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;

public class Robot extends TimedRobot {
  // ★ [수정] 카메라 2개 선언 (Main, Sub)
  // PhotonVision 대시보드에서 설정한 카메라 이름과 똑같이 적어야 합니다.
  PhotonCamera camera = new PhotonCamera("FHD_Webcam");
  PhotonCamera camera2 = new PhotonCamera("USB_CAMERA");

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private SparkMax leftMotor1 = new SparkMax(1, MotorType.kBrushed);
  private SparkMax leftMotor2 = new SparkMax(2, MotorType.kBrushed);
  private SparkMax rightMotor1 = new SparkMax(3, MotorType.kBrushed);
  private SparkMax rightMotor2 = new SparkMax(4, MotorType.kBrushed);

  private SparkMax Motor5 = new SparkMax(5, MotorType.kBrushed);
  private SparkMax Motor6 = new SparkMax(8, MotorType.kBrushed); // 6-> 8
  private SparkMax Motor7 = new SparkMax(6, MotorType.kBrushed); // 7-> 6
  private SparkMax Motor8 = new SparkMax(7, MotorType.kBrushed); // 8-> 7

  private Joystick joystick01 = new Joystick(0);
  private Joystick joystick02 = new Joystick(1);

  // ★ [추가 요청] 텔레옵 비전 어시스트용 버튼 설정 (5번: Trigger)
  private static final int kVisionButton = 5;

  // ★ [추가 요청] 거리 자동 조준 버튼 및 거리 설정
  private static final int kDistBtn1 = 1; // 버튼 1:왼쪽
  private static final int kDistBtn2 = 2; // 버튼 2: 아래
  private static final int kDistBtn3 = 3; // 버튼 3: 오른쪽
  private static final int kDistBtn4 = 4; // 버튼 4: 위

  private static final double kDistVal1 = 0.65; // 왼쪽 경사 기둥 거리
  private static final double kDistVal2 = 1.20; // 왼쪽 수직 기둥 거리
  private static final double kDistVal3 = 0.65; // 오른쪽 경사 기둥 거리
  private static final double kDistVal4 = 1.40; // 오른쪽 수직 기둥 거리

  // ★ [추가 요청] POV 정밀 제어 속도 상수
  private static final double kFineSpeed = 0.11;
  private static final double kFineTurn = 0.12;

  // 변수들
  private double leftStickY = 0;
  private double rightStickX = 0;
  private double leftStickX2 = 0;
  private double leftStickY2 = 0;
  private double rightStickX2 = 0;
  private double rightStickY2 = 0;

  private double speed = 0;
  private double turn = 0;
  private double leftPower = 0;
  private double rightPower = 0;
  private double forwardSpeed = 0;
  private double turnSpeed = 0;

  private double Motor5Speed = 0;
  private double Motor6Speed = 0;
  private double Motor7Speed = 0;
  private double Motor8Speed = 0;

  private DigitalInput digitalInput_0;
  private final Timer timer = new Timer();
  private boolean previousInputValue = false;

  // -------------------------------------------------------------------------
  // [물리적 측정값] - SmartDashboard 수정 가능 변수
  // -------------------------------------------------------------------------
  private double CAMERA_HEIGHT_METERS = 0.4;
  private double TARGET_HEIGHT_METERS = 0.27;
  // 각도 값을 대시보드에서 수정하기 위해 Degree 변수 사용
  private double CAMERA_PITCH_DEGREES = -11.29;

  // ★ [추가] 2번 카메라용 물리적 측정값 및 오프셋
  private double CAMERA2_HEIGHT_METERS = 0.63;
  private double CAMERA2_PITCH_DEGREES = -22.33;
  // 카메라 2가 로봇 중심에서 오른쪽으로 얼마나 떨어져 있는지 (미터 단위)
  // 예: 오른쪽으로 22.5cm 떨어져 있으면 0.225, 왼쪽이면 -0.225
  private double CAMERA2_LATERAL_OFFSET_METERS = 0.225;

  // -------------------------------------------------------------------------
  // [튜닝 상수] - SmartDashboard 수정 가능 변수
  // -------------------------------------------------------------------------

  // 오른쪽 모터가 더 느린 것을 보정
  private double RIGHT_MOTOR_CORRECTION = 1.0005;

  // 목표 지점의 거리 및 yaw 각도를 입력
  private double TARGET_DISTANCE_METERS = 0.825;
  private double TARGET_YAW_DEGREES_LEFT = 11.44;
  private double TARGET_YAW_DEGREES_RIGHT = -21.22;

  // P값 튜닝
  private double FORWARD_kP = 1.4;
  private double TURN_kP = 0.002;

  // 최소 기동 전압
  private double kMinForwardSpeed = 0.1;
  private double kMinTurnSpeed = 0.09;

  private double DISTANCE_TOLERANCE_METERS = 0.02;
  private double YAW_TOLERANCE_DEGREES = 1.0;

  private double MAX_FORWARD_SPEED = 0.10;
  private double MAX_TURN_SPEED = 0.10;

  // 안정화 시간
  private double STABLE_TIME_REQUIRED = 0.01;

  // ========================================================================
  // 자율주행 단계(Phase) 정의
  // ========================================================================
  private enum AutoPhase {
    BLIND_MOVE, // 1단계: 초기 딜레이 또는 무조건 전진
    APPROACH, // 2단계: 비전 추적 및 접근
    ACTION, // 3단계: 도착 후 동작 (파이프 배출)
    FINISHED // 4단계: 종료 (정지)
  }

  // 상태 관리 변수
  private AutoPhase currentPhase = AutoPhase.BLIND_MOVE;
  private SearchState searchState = SearchState.TRACKING;

  // 타이머 및 추적 변수
  private double lastKnownYaw = 0.0; // 마지막으로 본 타겟의 Yaw 각도
  private double targetLostTime = 0.0; // 타겟을 잃은 시각
  private double searchStartTime = 0.0; // 탐색 시작 시각
  private int searchSweepDirection = 1; // 스캔 방향 (1: 우회전, -1: 좌회전)

  private double atTargetTimer = 0.0; // 목표 지점 안정화 타이머 (오실레이션 방지용)
  private double actionTimer = 0.0; // 액션 수행 타이머
  // ★ [추가] 타겟 발견 시 안정화 타이머
  private double acquisitionTimer = 0.0;

  // 비전 탐색 상태 (APPROACH 단계 내부에서 사용)
  private enum SearchState {
    TRACKING, // 타겟 추적 중
    TARGET_LOST, // 타겟 방금 잃음
    SEARCHING_LAST_YAW, // 마지막 본 방향으로 회전
    SEARCHING_SWEEP // 좌우 스캔
  }

  // 탐색 모드 튜닝 상수
  private static final double SEARCH_TURN_SPEED = 0.20; // 탐색 시 회전 속도
  private static final double SEARCH_WAIT_TIME = 1.5; // 타겟 잃은 후 대기 시간 (초)
  private static final double SEARCH_YAW_TIME = 3.0; // 마지막 방향 탐색 시간 (초)
  private static final double SEARCH_SWEEP_DURATION = 6.0; // 좌우 스캔 지속 시간 (초)
  private static final double TARGET_LOST_TIMEOUT = 8.0; // 탐색 포기 시간 (초)

  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // 하드웨어 안전 초기화 (생성자)
    try {
      digitalInput_0 = new DigitalInput(0);
    } catch (Exception e) {
      System.out.println("DigitalInput 0 Init Failed: " + e.getMessage());
    }
  }

  @Override
  public void robotInit() {
    DataLogManager.start(); // 로그 기록 시작
    System.out.println("Robot Initialized & Logging Started");

    // 2025 REVLib 모터 설정
    SparkMaxConfig config = new SparkMaxConfig();

    config.voltageCompensation(12.0);
    config.openLoopRampRate(0.5);
    config.closedLoopRampRate(0.5);

    leftMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Motor8.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ★ [추가 요청] 타겟 ID 필터링 옵션 초기화
    SmartDashboard.putNumber("Target Filter ID", -1);

    // ★ [추가 요청] 튜닝 변수 SmartDashboard 초기화
    SmartDashboard.putNumber("Cam Height (m)", CAMERA_HEIGHT_METERS);
    SmartDashboard.putNumber("Target Height (m)", TARGET_HEIGHT_METERS);
    SmartDashboard.putNumber("Cam Pitch (Deg)", CAMERA_PITCH_DEGREES);

    // ★ [추가] 2번 카메라용 대시보드 변수
    SmartDashboard.putNumber("Cam2 Height (m)", CAMERA2_HEIGHT_METERS);
    SmartDashboard.putNumber("Cam2 Pitch (Deg)", CAMERA2_PITCH_DEGREES);
    SmartDashboard.putNumber("Cam2 Offset (m)", CAMERA2_LATERAL_OFFSET_METERS);

    SmartDashboard.putNumber("Right Motor Corr", RIGHT_MOTOR_CORRECTION);

    SmartDashboard.putNumber("Target Dist (m)", TARGET_DISTANCE_METERS);
    SmartDashboard.putNumber("Target Yaw Left", TARGET_YAW_DEGREES_LEFT);
    SmartDashboard.putNumber("Target Yaw Right", TARGET_YAW_DEGREES_RIGHT);

    SmartDashboard.putNumber("Forward kP", FORWARD_kP);
    SmartDashboard.putNumber("Turn kP", TURN_kP);

    SmartDashboard.putNumber("Min Fwd Speed", kMinForwardSpeed);
    SmartDashboard.putNumber("Min Turn Speed", kMinTurnSpeed);

    SmartDashboard.putNumber("Dist Tolerance", DISTANCE_TOLERANCE_METERS);
    SmartDashboard.putNumber("Yaw Tolerance", YAW_TOLERANCE_DEGREES);

    SmartDashboard.putNumber("Max Fwd Speed", MAX_FORWARD_SPEED);
    SmartDashboard.putNumber("Max Turn Speed", MAX_TURN_SPEED);

    SmartDashboard.putNumber("Stable Time", STABLE_TIME_REQUIRED);
  }

  // ★ [핵심] 매 주기마다 대시보드 값 업데이트
  @Override
  public void robotPeriodic() {
    CAMERA_HEIGHT_METERS = SmartDashboard.getNumber("Cam Height (m)", CAMERA_HEIGHT_METERS);
    TARGET_HEIGHT_METERS = SmartDashboard.getNumber("Target Height (m)", TARGET_HEIGHT_METERS);
    CAMERA_PITCH_DEGREES = SmartDashboard.getNumber("Cam Pitch (Deg)", CAMERA_PITCH_DEGREES);

    // 2번 카메라 설정값 업데이트
    CAMERA2_HEIGHT_METERS = SmartDashboard.getNumber("Cam2 Height (m)", CAMERA2_HEIGHT_METERS);
    CAMERA2_PITCH_DEGREES = SmartDashboard.getNumber("Cam2 Pitch (Deg)", CAMERA2_PITCH_DEGREES);
    CAMERA2_LATERAL_OFFSET_METERS = SmartDashboard.getNumber("Cam2 Offset (m)", CAMERA2_LATERAL_OFFSET_METERS);

    RIGHT_MOTOR_CORRECTION = SmartDashboard.getNumber("Right Motor Corr", RIGHT_MOTOR_CORRECTION);

    TARGET_DISTANCE_METERS = SmartDashboard.getNumber("Target Dist (m)", TARGET_DISTANCE_METERS);
    TARGET_YAW_DEGREES_LEFT = SmartDashboard.getNumber("Target Yaw Left", TARGET_YAW_DEGREES_LEFT);
    TARGET_YAW_DEGREES_RIGHT = SmartDashboard.getNumber("Target Yaw Right", TARGET_YAW_DEGREES_RIGHT);

    FORWARD_kP = SmartDashboard.getNumber("Forward kP", FORWARD_kP);
    TURN_kP = SmartDashboard.getNumber("Turn kP", TURN_kP);

    kMinForwardSpeed = SmartDashboard.getNumber("Min Fwd Speed", kMinForwardSpeed);
    kMinTurnSpeed = SmartDashboard.getNumber("Min Turn Speed", kMinTurnSpeed);

    DISTANCE_TOLERANCE_METERS = SmartDashboard.getNumber("Dist Tolerance", DISTANCE_TOLERANCE_METERS);
    YAW_TOLERANCE_DEGREES = SmartDashboard.getNumber("Yaw Tolerance", YAW_TOLERANCE_DEGREES);

    MAX_FORWARD_SPEED = SmartDashboard.getNumber("Max Fwd Speed", MAX_FORWARD_SPEED);
    MAX_TURN_SPEED = SmartDashboard.getNumber("Max Turn Speed", MAX_TURN_SPEED);

    STABLE_TIME_REQUIRED = SmartDashboard.getNumber("Stable Time", STABLE_TIME_REQUIRED);
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    timer.reset();
    timer.start();

    // ★ 상태 초기화
    currentPhase = AutoPhase.BLIND_MOVE;
    searchState = SearchState.TRACKING;
    atTargetTimer = 0.0;
    actionTimer = 0.0;

    // ★ 탐색 상태 초기화

    lastKnownYaw = 0.0;
    targetLostTime = 0.0;
    searchStartTime = 0.0;
    searchSweepDirection = 1;
  }

  @Override
  public void autonomousPeriodic() {
    // 현재 단계(Phase)에 따라 다른 동작 수행
    switch (currentPhase) {
      // ----------------------------------------------------------
      // 1단계: 블라인드 무브 (초기 2초)
      // ----------------------------------------------------------
      case BLIND_MOVE:
        if (timer.get() < 5) {
          // 사용자 코드 기준: 0.0 (정지 상태로 대기)
          // 만약 전진이 필요하면 0.1 등으로 수정하세요.
          runDriveMotors(0.19, 0);
          Motor5.set(-1.0);
          System.out.println("PHASE: BLIND MOVE (Waiting...)");
        } else {
          // 2초 지나면 비전 접근 단계로 전환
          Motor5.set(0);
          System.out.println("-> Switching to APPROACH Phase");
          currentPhase = AutoPhase.APPROACH;
        }
        break;

      // ----------------------------------------------------------
      // 2단계: 비전 추적 및 접근
      // ----------------------------------------------------------
      case APPROACH:
        runVisionApproach(); // 안정화 로직이 포함된 비전 주행
        break;

      // ----------------------------------------------------------
      // 3단계: 도착 후 동작 (파이프 배출)
      // ----------------------------------------------------------
      case ACTION:
        runActionSequence();
        break;

      // ----------------------------------------------------------
      // 4단계: 종료
      // ----------------------------------------------------------
      case FINISHED:
        runDriveMotors(0, 0); // 구동 모터 정지
        stopAllMechanismMotors(); // 기구부 모터 정지
        // System.out.println("Auto Finished.");
        break;
    }

    // 상태 대시보드 출력
    SmartDashboard.putString("Auto/Phase", currentPhase.toString());
  }

  // ★ [핵심] 비전 접근 로직 (안정화 타이머 포함)
  private void runVisionApproach() {
    // 1. 카메라 1 결과 확인
    PhotonPipelineResult result1 = camera.getLatestResult();
    // 2. 카메라 2 결과 확인
    PhotonPipelineResult result2 = camera2.getLatestResult();

    PhotonTrackedTarget target = null;
    boolean useCamera2 = false; // 현재 사용중인 카메라가 2번인지 여부

    // ★ [추가 요청 반영] 타겟 ID 필터링 로직
    double filterId = SmartDashboard.getNumber("Target Filter ID", -1);

    // ----------------------------------------
    // 카메라 선택 로직: Main -> Sub 순서로 탐색
    // ----------------------------------------

    // (1) Main Camera 검사
    if (result1.hasTargets()) {
      if (filterId == -1) {
        target = result1.getBestTarget();
      } else {
        for (PhotonTrackedTarget t : result1.getTargets()) {
          if (t.getFiducialId() == (int) filterId) {
            target = t;
            break;
          }
        }
      }
    }

    // (2) Main에 타겟이 없고, Sub Camera 검사
    if (target == null && result2.hasTargets()) {
      if (filterId == -1) {
        target = result2.getBestTarget();
        useCamera2 = true;
      } else {
        for (PhotonTrackedTarget t : result2.getTargets()) {
          if (t.getFiducialId() == (int) filterId) {
            target = t;
            useCamera2 = true;
            break;
          }
        }
      }
    }

    // target 변수가 null이 아니면 타겟을 찾은 것으로 간주
    boolean hasTarget = (target != null);

    // 타겟이 보일 때
    if (hasTarget) {
      // ★ [문제 해결] 타겟을 탐색 모드에서 방금 찾았을 때
      if (searchState != SearchState.TRACKING) {
        forwardSpeed = 0;
        turnSpeed = 0;
        acquisitionTimer += 0.02; // 타이머 증가

        // 0.5초 동안 멈춰서 관성을 죽이고 타겟 인식을 확실히 함
        if (acquisitionTimer < 0.5) {
          System.out.println("Target Acquired! Stabilizing...");
          runDriveMotors(0, 0);
          return; // 여기서 리턴하여 PID 실행 안 함
        } else {
          // 0.5초 지났으면 추적 모드로 전환
          System.out.println("✓ TARGET LOCKED (" + (useCamera2 ? "CAM2" : "CAM1") + ") - Starting PID");
          searchState = SearchState.TRACKING;
          acquisitionTimer = 0.0;
        }
      }

      // --- 정상 추적 (PID 제어) ---
      double yaw = target.getYaw();
      double pitch = target.getPitch();

      // ★ 사용된 카메라에 따라 거리 계산 파라미터 자동 선택
      double currentCamHeight = useCamera2 ? CAMERA2_HEIGHT_METERS : CAMERA_HEIGHT_METERS;
      double currentCamPitch = useCamera2 ? CAMERA2_PITCH_DEGREES : CAMERA_PITCH_DEGREES;

      double distance = PhotonUtils.calculateDistanceToTargetMeters(
          currentCamHeight, TARGET_HEIGHT_METERS, Units.degreesToRadians(currentCamPitch),
          Units.degreesToRadians(pitch));

      // [수정] 거리 측정값이 음수(-)일 경우 예외 처리
      if (distance < 0) {
        distance = TARGET_DISTANCE_METERS + 2.0;
      }

      // ★ [핵심] 카메라 오프셋 보정 (Camera 2가 우측에 있을 경우)
      // 우측 카메라(양수 오프셋)로 타겟을 보면 타겟은 카메라 화면 왼쪽(양수 Yaw)에 위치함.
      // 로봇 중심을 맞추려면 이 "기하학적 각도"만큼 빼줘야 함.
      if (useCamera2) {
        double offsetCorrection = Units.radiansToDegrees(Math.atan(CAMERA2_LATERAL_OFFSET_METERS / distance));
        yaw -= offsetCorrection;
      }

      lastKnownYaw = yaw; // 보정된 Yaw를 저장해야 탐색 시 유리함

      double distError = distance - TARGET_DISTANCE_METERS;
      double yawError = yaw - TARGET_YAW_DEGREES_LEFT; // 왼쪽 기둥

      // PID 계산
      double fwdCmd = FORWARD_kP * distError;
      double turnCmd = -TURN_kP * yawError;

      // ★ [문제 해결] 목표 지점 안정화 확인 (Oscillation 방지)
      boolean isDistanceGood = Math.abs(distError) < DISTANCE_TOLERANCE_METERS;
      boolean isYawGood = Math.abs(yawError) < YAW_TOLERANCE_DEGREES;

      if (isDistanceGood && isYawGood) {
        // 오차 범위 내에 들어오면 모터 끄고 타이머 증가
        forwardSpeed = 0;
        turnSpeed = 0;
        atTargetTimer += 0.02; // 20ms 주기

        System.out.printf("In Range... Stable Time: %.2f/%.2f\n", atTargetTimer, STABLE_TIME_REQUIRED);

        // 안정화 시간(0.5초)을 채우면 ACTION 단계로 이동
        if (atTargetTimer >= STABLE_TIME_REQUIRED) {
          System.out.println("-> Target Reached! Switching to ACTION Phase");
          actionTimer = 0; // 액션 타이머 리셋
          currentPhase = AutoPhase.ACTION;
          return;
        }
      } else {
        // 오차 범위를 벗어나면 타이머 리셋 및 PID 제어 계속
        atTargetTimer = 0.0;
        forwardSpeed = fwdCmd + Math.copySign(kMinForwardSpeed, fwdCmd);
        turnSpeed = turnCmd + Math.copySign(kMinTurnSpeed, turnCmd);
      }

      // 속도 제한
      forwardSpeed = Math.max(-MAX_FORWARD_SPEED, Math.min(forwardSpeed, MAX_FORWARD_SPEED));
      turnSpeed = Math.max(-MAX_TURN_SPEED, Math.min(turnSpeed, MAX_TURN_SPEED));

    } else {
      // 타겟 없음: 탐색 로직 수행
      atTargetTimer = 0.0;
      handleSearchMode();
    }

    // 모터 구동
    runDriveMotors(forwardSpeed, turnSpeed);
  }

  // ★ [핵심] 도착 후 동작 로직 (파이프 배출)
  private void runActionSequence() {
    // 로봇 구동은 정지
    runDriveMotors(0, 0);

    // 액션 타이머 증가
    actionTimer += 0.01;

    // 1초 동안 Motor8 을 작동시켜 파이프를 배출
    if (actionTimer < 0.7) {
      Motor8.set(0.3);
      System.out.println("PERFORMING ACTION: Ejecting Object... " + String.format("%.2f", actionTimer));
    } else {
      // 동작 끝 -> 종료 단계로
      Motor8.set(0);
      System.out.println("-> Action Complete. Finishing Auto.");
      currentPhase = AutoPhase.FINISHED;
    }
  }

  // 탐색 모드 핸들러
  private void handleSearchMode() {

    double currentTime = Timer.getFPGATimestamp();
    switch (searchState) {
      case TRACKING:
        targetLostTime = currentTime;
        searchState = SearchState.TARGET_LOST;
        forwardSpeed = 0;
        turnSpeed = 0;
        break;
      case TARGET_LOST:
        if (currentTime - targetLostTime > SEARCH_WAIT_TIME) {
          searchStartTime = currentTime;
          searchState = SearchState.SEARCHING_LAST_YAW;
        }
        forwardSpeed = 0;
        turnSpeed = 0;
        break;
      case SEARCHING_LAST_YAW:
        if (currentTime - searchStartTime > SEARCH_YAW_TIME) {
          searchStartTime = currentTime;
          searchSweepDirection = (lastKnownYaw > 0) ? 1 : -1;
          searchState = SearchState.SEARCHING_SWEEP;
        }
        forwardSpeed = 0;
        turnSpeed = Math.copySign(SEARCH_TURN_SPEED, lastKnownYaw);
        break;
      case SEARCHING_SWEEP:
        if (currentTime - searchStartTime > SEARCH_SWEEP_DURATION) {
          searchSweepDirection *= -1;
          searchStartTime = currentTime;
        }
        if (currentTime - targetLostTime > TARGET_LOST_TIMEOUT) {
          forwardSpeed = 0;
          turnSpeed = 0;
        } else {
          forwardSpeed = 0;
          turnSpeed = SEARCH_TURN_SPEED * searchSweepDirection;
        }
        break;
    }
  }

  // 모터 구동 헬퍼 함수
  private void runDriveMotors(double fwd, double rot) {
    double l = fwd - rot;
    // 사용자 코드의 모터 방향(우측 양수, 좌측 음수) 반영
    double r = (fwd + rot) * RIGHT_MOTOR_CORRECTION;

    // Clamp
    l = Math.max(-1.0, Math.min(1.0, l));
    r = Math.max(-1.0, Math.min(1.0, r));

    leftMotor1.set(-l);
    leftMotor2.set(-l);
    rightMotor1.set(r);
    rightMotor2.set(r);
  }

  private void stopAllMechanismMotors() {
    Motor5.set(0);
    Motor6.set(0);
    Motor7.set(0);
    Motor8.set(0);
  }

  @Override
  public void teleopInit() {
    timer.start();
    if (digitalInput_0 != null)
      previousInputValue = digitalInput_0.get();
  }

  @Override
  public void teleopPeriodic() {
    // 1. Mechanism Controls (Joystick 2)
    leftStickX2 = joystick02.getRawAxis(0);
    leftStickY2 = joystick02.getRawAxis(1);
    rightStickX2 = joystick02.getRawAxis(2);
    rightStickY2 = joystick02.getRawAxis(5);
    int pov2 = joystick02.getPOV();
    boolean btn2L = joystick02.getRawButton(7);
    boolean btn2B = joystick02.getRawButton(2);
    boolean btn2R = joystick02.getRawButton(8);
    boolean btn2F = joystick02.getRawButton(4);

    if (Math.abs(leftStickX2) < 0.3)
      leftStickX2 = 0;
    if (Math.abs(leftStickY2) < 0.3)
      leftStickY2 = 0;
    if (Math.abs(rightStickX2) < 0.3)
      rightStickX2 = 0;
    if (Math.abs(rightStickY2) < 0.5)
      rightStickY2 = 0;

    Motor5Speed = leftStickY2 * 1.0;
    if (leftStickX2 > 0)
      Motor6Speed = -leftStickX2 * 0.5;
    else
      Motor6Speed = leftStickX2 * 0.5;

    if (rightStickX2 > 0)
      Motor7Speed = -rightStickY2 * 1.0;
    else
      Motor7Speed = -rightStickY2 * 0.3;

    Motor8Speed = rightStickX2 * 0.5;

    if (pov2 != -1) {

      if (pov2 == 0) {
        Motor5Speed = -0.5;
      }
      if (pov2 == 180) {
        Motor5Speed = 0.5;
      }
      if (pov2 == 90 || pov2 == 270) {
        Motor6Speed = -0.5;
      }
    }
    if (btn2F)
      Motor7Speed = 0.5;
    if (btn2B)
      Motor7Speed = -0.3;
    if (btn2L)
      Motor8Speed = -0.3;
    if (btn2R)
      Motor8Speed = 0.5;

    Motor5.set(Motor5Speed);
    Motor6.set(Motor6Speed);
    Motor7.set(Motor7Speed);
    Motor8.set(Motor8Speed);

    // 2. Drive Inputs & Vision (Joystick 1)
    leftStickY = joystick01.getRawAxis(2);
    rightStickX = joystick01.getRawAxis(1);

    // ----------------------------------------
    // Teleop Vision Logic (Dual Camera)
    // ----------------------------------------
    PhotonPipelineResult result1 = camera.getLatestResult();
    PhotonPipelineResult result2 = camera2.getLatestResult();

    PhotonTrackedTarget target = null;
    boolean useCamera2 = false;

    // Filter ID 적용
    double filterId = SmartDashboard.getNumber("Target Filter ID", -1);

    // (1) Check Main Camera
    if (result1.hasTargets()) {
      if (filterId == -1) {
        target = result1.getBestTarget();
      } else {
        for (PhotonTrackedTarget t : result1.getTargets()) {
          if (t.getFiducialId() == (int) filterId) {
            target = t;
            break;
          }
        }
      }
    }

    // (2) Check Sub Camera if Main failed
    if (target == null && result2.hasTargets()) {
      if (filterId == -1) {
        target = result2.getBestTarget();
        useCamera2 = true;
      } else {
        for (PhotonTrackedTarget t : result2.getTargets()) {
          if (t.getFiducialId() == (int) filterId) {
            target = t;
            useCamera2 = true;
            break;
          }
        }
      }
    }

    boolean hasTarget = (target != null);

    // 거리/Yaw 계산
    double currentDistance = 0.0;
    double currentYaw = 0.0;
    double targetID = -1;

    if (hasTarget) {
      currentYaw = target.getYaw();
      double pitch = target.getPitch();
      targetID = target.getFiducialId();

      double currentCamHeight = useCamera2 ? CAMERA2_HEIGHT_METERS : CAMERA_HEIGHT_METERS;
      double currentCamPitch = useCamera2 ? CAMERA2_PITCH_DEGREES : CAMERA_PITCH_DEGREES;

      currentDistance = PhotonUtils.calculateDistanceToTargetMeters(
          currentCamHeight, TARGET_HEIGHT_METERS, Units.degreesToRadians(currentCamPitch),
          Units.degreesToRadians(pitch));

      if (currentDistance < 0) {
        currentDistance = TARGET_DISTANCE_METERS + 1.0;
      }

      // ★ [핵심] Teleop에서도 카메라 오프셋 보정 적용
      if (useCamera2) {
        double offsetCorrection = Units.radiansToDegrees(Math.atan(CAMERA2_LATERAL_OFFSET_METERS / currentDistance));
        currentYaw -= offsetCorrection;
      }
    }

    // 3. Control Mode Selection
    // 버튼 상태 확인
    boolean btnDist1 = joystick01.getRawButton(kDistBtn1);
    boolean btnDist2 = joystick01.getRawButton(kDistBtn2);
    boolean btnDist3 = joystick01.getRawButton(kDistBtn3);
    boolean btnDist4 = joystick01.getRawButton(kDistBtn4);
    boolean btnVisionAssist = joystick01.getRawButton(kVisionButton);

    // 자동 거리 조절 모드인지 확인
    boolean isAutoDistanceMode = (btnDist1 || btnDist2 || btnDist3 || btnDist4);
    double targetDistanceSetpoint = TARGET_DISTANCE_METERS;

    if (isAutoDistanceMode && hasTarget) {
      // 3-1. Full Auto (Distance + Turn)
      if (btnDist1) {
        targetDistanceSetpoint = kDistVal1;
        double yawError = currentYaw - TARGET_YAW_DEGREES_LEFT;
        double distError = currentDistance - targetDistanceSetpoint;
        double fwdCmd = FORWARD_kP * distError;
        double turnCmd = -TURN_kP * yawError;
        speed = Math.max(-MAX_FORWARD_SPEED, Math.min(fwdCmd, MAX_FORWARD_SPEED));
        turn = Math.max(-MAX_TURN_SPEED, Math.min(turnCmd, MAX_TURN_SPEED));

      } else if (btnDist2) {
        targetDistanceSetpoint = kDistVal2;
        double yawError = currentYaw - TARGET_YAW_DEGREES_LEFT;
        double distError = currentDistance - targetDistanceSetpoint;
        double fwdCmd = FORWARD_kP * distError;
        double turnCmd = -TURN_kP * yawError;
        speed = Math.max(-MAX_FORWARD_SPEED, Math.min(fwdCmd, MAX_FORWARD_SPEED));
        turn = Math.max(-MAX_TURN_SPEED, Math.min(turnCmd, MAX_TURN_SPEED));
      } else if (btnDist3) {
        targetDistanceSetpoint = kDistVal3;
        double yawError = currentYaw - TARGET_YAW_DEGREES_RIGHT;
        double distError = currentDistance - targetDistanceSetpoint;
        double fwdCmd = FORWARD_kP * distError;
        double turnCmd = -TURN_kP * yawError;
        speed = Math.max(-MAX_FORWARD_SPEED, Math.min(fwdCmd, MAX_FORWARD_SPEED));
        turn = Math.max(-MAX_TURN_SPEED, Math.min(turnCmd, MAX_TURN_SPEED));
      } else if (btnDist4) {
        targetDistanceSetpoint = kDistVal4;
        double yawError = currentYaw - TARGET_YAW_DEGREES_RIGHT;
        double distError = currentDistance - targetDistanceSetpoint;
        double fwdCmd = FORWARD_kP * distError;
        double turnCmd = -TURN_kP * yawError;
        speed = Math.max(-MAX_FORWARD_SPEED, Math.min(fwdCmd, MAX_FORWARD_SPEED));
        turn = Math.max(-MAX_TURN_SPEED, Math.min(turnCmd, MAX_TURN_SPEED));
      }

      // 최소 구동력 보정
      if (Math.abs(speed) < kMinForwardSpeed && Math.abs(speed) > 0.01)
        speed = Math.copySign(kMinForwardSpeed, speed);
      if (Math.abs(turn) < kMinTurnSpeed && Math.abs(turn) > 0.01)
        turn = Math.copySign(kMinTurnSpeed, turn);

      SmartDashboard.putString("DriveMode", "AutoDistance: " + targetDistanceSetpoint + "m");

      // 4. Apply Drive to Motors
      leftPower = speed - turn;
      if ((speed + turn) > 0)
        rightPower = (speed + turn) * (RIGHT_MOTOR_CORRECTION * 1.05);
      else
        rightPower = (speed + turn) * RIGHT_MOTOR_CORRECTION;

      leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
      rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

      leftMotor1.set(-leftPower);
      leftMotor2.set(-leftPower);
      rightMotor1.set(rightPower);
      rightMotor2.set(rightPower);

    } else if (btnVisionAssist && hasTarget) {
      // 3-2. Vision Assist (Manual Speed + Auto Turn)
      // 전진: 수동
      double rawSpeed = (Math.abs(leftStickY) < 0.05) ? 0 : leftStickY;
      speed = Math.copySign(rawSpeed * rawSpeed, rawSpeed) * 0.7;

      // 회전: 자동
      double yawError = currentYaw - TARGET_YAW_DEGREES_LEFT;
      double turnCmd = -TURN_kP * yawError;
      turn = Math.max(-MAX_TURN_SPEED, Math.min(turnCmd, MAX_TURN_SPEED));

      if (Math.abs(turn) < kMinTurnSpeed && Math.abs(turn) > 0.005) {
        turn = Math.copySign(kMinTurnSpeed, turn);
      }
      SmartDashboard.putString("DriveMode", "VisionAssist");

      // 4. Apply Drive to Motors
      leftPower = speed - turn;
      if ((speed + turn) > 0)
        rightPower = (speed + turn) * (RIGHT_MOTOR_CORRECTION * 1.05);
      else
        rightPower = (speed + turn) * RIGHT_MOTOR_CORRECTION;

      leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
      rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

      leftMotor1.set(-leftPower);
      leftMotor2.set(-leftPower);
      rightMotor1.set(rightPower);
      rightMotor2.set(rightPower);

    } else {
      // 3-3. Manual Control
      double rawSpeed = (Math.abs(leftStickY) < 0.01) ? 0 : leftStickY;
      double rawTurn = (Math.abs(rightStickX) < 0.01) ? 0 : rightStickX;

      speed = Math.copySign(rawSpeed, rawSpeed) * 0.7;
      turn = Math.copySign(rawTurn, rawTurn) * 0.7;
      // ★ [추가] POV(D-pad) 정밀 제어 오버라이드
      // POV 버튼을 누르면 기존 아날로그 스틱 값을 무시하고 정밀 속도로 덮어씁니다.
      int pov = joystick01.getPOV();
      if (pov != -1) {
        // POV 사용 시 속도 리셋
        speed = 0;
        turn = 0;

        // 전진 (0, 45, 315)
        if (pov == 180) {
          // speed = kFineSpeed;
          turn = kFineTurn;
        }
        // 후진 (180, 135, 225)
        else if (pov == 0) {
          // speed = -kFineSpeed;
          turn = -kFineTurn;
        }

        // 우회전 (90, 45, 135) -> Turn값 음수
        if (pov == 270) {
          speed = -kFineSpeed;
        }
        // 좌회전 (270, 225, 315) -> Turn값 양수
        else if (pov == 90) {
          speed = kFineSpeed;

        }

        SmartDashboard.putString("DriveMode", "Manual (POV)");
      } else {
        SmartDashboard.putString("DriveMode", "Manual");
      }

      // 3-4. Apply Drive to Motors
      leftPower = speed - turn;
      if ((speed + turn) > 0)
        rightPower = (speed + turn) * (RIGHT_MOTOR_CORRECTION * 1.05);
      else
        rightPower = (speed + turn) * RIGHT_MOTOR_CORRECTION;

      leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
      rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

      leftMotor1.set(-leftPower);
      leftMotor2.set(-leftPower);
      rightMotor1.set(-rightPower);
      rightMotor2.set(-rightPower);
    }

    // 5. Logging & Digital Input
    if (hasTarget)

    {
      if (searchState != SearchState.TRACKING) {
        searchState = SearchState.TRACKING;
      }
      // 로그 출력 (사용중인 카메라 표시)
      System.out.printf("[%s] Mode:%s | Cam:%s | Dist:%.2f | Yaw:%.2f | ID:%.0f\n",
          searchState, (isAutoDistanceMode ? "AUTO-DIST" : "MANUAL"), (useCamera2 ? "SUB" : "MAIN"), currentDistance,
          currentYaw, targetID);
    }

    if (digitalInput_0 != null) {
      boolean currentInputValue = digitalInput_0.get();
      if (currentInputValue != previousInputValue) {
        System.out.println("Input state changed to " + (currentInputValue ? "ON" : "OFF"));
        timer.reset();
        previousInputValue = currentInputValue;
      }
    }
  }

  @Override
  public void disabledInit() {
    leftMotor1.set(0);
    leftMotor2.set(0);
    rightMotor1.set(0);
    rightMotor2.set(0);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    timer.reset();
    timer.start();
  }

  @Override
  public void testPeriodic() {
    // 1. 로봇을 타겟 정면 정확히 1.0m 거리에 둡니다.
    double realDist = 1.0; // 실제 거리 (미터)

    // ----------------------------------------
    // Camera 1 (Main) Pitch Calibration
    // ----------------------------------------
    var result1 = camera.getLatestResult();
    if (result1.hasTargets()) {
      double targetPitchDeg = result1.getBestTarget().getPitch();
      double targetPitchRad = Units.degreesToRadians(targetPitchDeg);

      // 공식: pitch_cam = arctan((h_target - h_cam) / dist) - pitch_target
      // SmartDashboard에서 설정된 카메라 높이 사용
      double heightDiff = TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS;
      double totalAngleRad = Math.atan(heightDiff / realDist);
      double calcCamPitchRad = totalAngleRad - targetPitchRad;
      double calcCamPitchDeg = Units.radiansToDegrees(calcCamPitchRad);

      System.out.printf("[CAM 1] Current Target Pitch: %.2f | >>> REQUIRED PITCH: %.2f (Update SmartDashboard!)\n",
          targetPitchDeg, calcCamPitchDeg);
    }

    // ----------------------------------------
    // Camera 2 (Sub) Pitch Calibration
    // ----------------------------------------
    var result2 = camera2.getLatestResult();
    if (result2.hasTargets()) {
      double targetPitchDeg = result2.getBestTarget().getPitch();
      double targetPitchRad = Units.degreesToRadians(targetPitchDeg);

      // SmartDashboard에서 설정된 2번 카메라 높이 사용
      double heightDiff = TARGET_HEIGHT_METERS - CAMERA2_HEIGHT_METERS;
      double totalAngleRad = Math.atan(heightDiff / realDist);
      double calcCamPitchRad = totalAngleRad - targetPitchRad;
      double calcCamPitchDeg = Units.radiansToDegrees(calcCamPitchRad);

      System.out.printf("[CAM 2] Current Target Pitch: %.2f | >>> REQUIRED PITCH: %.2f (Update SmartDashboard!)\n",
          targetPitchDeg, calcCamPitchDeg);
    }
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
    // 1. 로봇을 타겟 정면 정확히 1.0m (또는 2.0m) 거리에 둡니다.
    // 2. 줄자로 잰 실제 높이들을 입력합니다.
    double realDist = 1.0; // 실제 거리 (미터)
    double camHeight = 0.31; // 실제 카메라 높이
    double targetHeight = 0.27; // 실제 타겟 높이

    // 3. PhotonVision에서 현재 보이는 타겟의 피치 값을 가져옵니다.
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      double targetPitchDeg = result.getBestTarget().getPitch();
      double targetPitchRad = Units.degreesToRadians(targetPitchDeg);

      // 4. 역산 공식 (아크탄젠트 사용)
      // 공식: pitch_cam = arctan((h_target - h_cam) / dist) - pitch_target
      double heightDiff = targetHeight - camHeight;
      double totalAngleRad = Math.atan(heightDiff / realDist);
      double calcCamPitchRad = totalAngleRad - targetPitchRad;

      // 5. 결과 출력 (이 값을 코드의 상수로 쓰세요!)
      double calcCamPitchDeg = Units.radiansToDegrees(calcCamPitchRad);

      System.out.printf(">>> REQUIRED PITCH: %.2f Degrees (Put this in constant!)\n", calcCamPitchDeg);
      System.out.printf(">>> Current Pitch from Target: %.2f\n", targetPitchDeg);
    }
  }
}
