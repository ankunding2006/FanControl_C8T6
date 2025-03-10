#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "key.h"
#include "oled.h"
#include "fan_driver.h"
#include "angle_sensor.h"
#include "angle_control.h"
#include "pid_controller.h"
#include <string.h>
#include <math.h>

/* 系统状态定义 */
typedef enum
{
    STATE_MENU = 0,      // 菜单状态
    STATE_ANGLE_SETTING, // 角度设置状态
    STATE_RUNNING,       // 运行状态
    STATE_PARAM_SETTING  // 参数设置状态
} SystemState_TypeDef;

/* 工作模式定义 */
typedef enum
{
    MODE_IDLE = 0,         // 空闲模式
    MODE_SINGLE_FAN_45DEG, // 单风扇45度模式
    MODE_SINGLE_FAN_ANY,   // 单风扇任意角度模式
    MODE_DUAL_FAN_ANY,     // 双风扇任意角度模式
    MODE_DUAL_FAN_SEQUENCE // 双风扇序列模式
} WorkMode_TypeDef;

/* 全局变量 */
AngleControl_TypeDef g_angle_control;     // 角度控制结构体
static SystemState_TypeDef g_systemState; // 系统状态
static WorkMode_TypeDef g_workMode;       // 工作模式
static float g_targetAngle = 0.0f;        // 目标角度
static uint32_t g_modeStartTime = 0;      // 模式开始时间

/* 显示缓冲区，用于存储上一次显示的内容 */
static char g_lastDisplayBuf[8][32];                       // 8行显示，每行最多32个字符
static SystemState_TypeDef g_lastSystemState = STATE_MENU; // 上一次的系统状态，初始化为菜单状态
static WorkMode_TypeDef g_lastWorkMode = MODE_IDLE;        // 上一次的工作模式，初始化为空闲模式
static float g_lastTargetAngle = -1.0f;                    // 上一次显示的目标角度
static float g_lastCurrentAngle = -1.0f;                   // 上一次显示的当前角度
static uint32_t g_lastElapsedTime = 0xFFFFFFFF;            // 上一次显示的经过时间

/* 函数声明 */
static void System_Init(void);
static void Timer_Init(void);
static void UserInterface_Process(void);
static void ProcessKeys(void);
static void MenuManager(void);
static void ConfigureControlMode(WorkMode_TypeDef mode);
void DisplayStatus(void);
void ShowBootAnimation(void);

/**
 * @brief  系统初始化函数
 * @param  无
 * @retval 无
 */
static void System_Init(void)
{
    // 配置中断分组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // 基础系统初始化
    delay_init();
    NVIC_SetPriority(SysTick_IRQn, 0); // 最高优先级
    uart_init(115200);

    // 初始化各外设
    KEY_Init();
    ANGLE_SENSOR_Init();
    FAN_Init();

    // 初始化角度控制系统
    ANGLE_CONTROL_Init(&g_angle_control, CONTROL_MODE_IDLE);

    // PID参数初始化 - 为角度控制器配置适当的参数
    PID_Init(&(g_angle_control.pid), 2.0f, 0.05f, 1.0f, PID_MODE_POSITION, 0.01f);

    // 初始化定时器
    Timer_Init();

    // 初始化系统状态和变量
    g_systemState = STATE_MENU;
    g_workMode = MODE_IDLE;
    g_targetAngle = 0.0f;
    g_modeStartTime = 0;

    // 初始化显示缓冲区
    memset(g_lastDisplayBuf, 0, sizeof(g_lastDisplayBuf));

    // 使用硬件看门狗防止系统死锁
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    // Configure IWDG with maximum timeout
    IWDG_SetReload(0xFFF);
    IWDG_ReloadCounter();
    // IWDG_Enable();

    // 输出初始化完成信息
    printf("System initialization complete\r\n");
}

/**
 * @brief  定时器初始化函数
 * @param  无
 * @retval 无
 */
static void Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能定时器时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    // TIM3配置 - 10ms中断
    TIM_TimeBaseStructure.TIM_Period = 9999;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // TIM4配置 - 1ms中断
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = 71; // 明确指定预分频值
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // 配置NVIC - TIM3
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 配置NVIC - TIM4
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);

    // 使能定时器中断
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    // 启动定时器
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/**
 * @brief  主函数
 * @param  无
 * @retval int
 */
int main(void)
{
    // 系统初始化
    System_Init();
    delay_ms(100);
    OLED_Init();
    OLED_ColorTurn(0);   // 0正常显示，1 反色显示
    OLED_DisplayTurn(0); // 0正常显示 1 屏幕翻转显示

    // 播放开机动画
    ShowBootAnimation();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"Wind Panel Control", 12, 1); // 显示欢迎信息
    OLED_ShowString(0, 12, (u8 *)"System Ready", 12, 1);
    OLED_Refresh(); // 确保信息显示后刷新
    printf("Wind Panel Control System Started\r\n");
    delay_ms(1500);
    // 主循环
    while (1)
    {

        UserInterface_Process(); // 用户交互处理
        if (g_systemState == STATE_RUNNING)
        {
            printf("Angle: %.2f\r\n", ANGLE_SENSOR_GetAngle());
        }
        DisplayStatus(); // 显示状态更新
    }
}

/**
 * @brief  用户界面处理函数
 * @param  无
 * @retval 无
 */
static void UserInterface_Process(void)
{
    ProcessKeys();
    MenuManager();
}

/**
 * @brief  按键处理函数
 * @param  无
 * @retval 无
 */
static void ProcessKeys(void)
{
    uint8_t key = KEY_Scan();
    if (key == KEY_NONE)
        return;
    printf("Key: %d\r\n", key);
    switch (g_systemState)
    {

    case STATE_MENU:
        // 模式选择
        if (key == KEY_UP || key == KEY_DOWN)
        {
            if (key == KEY_UP)
                g_workMode++;
            else
                g_workMode--;

            // 删除重复的条件判断，只保留完整的逻辑
            if (g_workMode > MODE_DUAL_FAN_SEQUENCE)
            {
                g_workMode = MODE_IDLE;
            }
            else if (g_workMode == MODE_IDLE && key == KEY_DOWN)
            {
                g_workMode = MODE_DUAL_FAN_SEQUENCE;
            }
        }
        else if (key == KEY_ENTER)
        {
            // 进入角度设置或直接开始
            if (g_workMode == MODE_SINGLE_FAN_45DEG)
            {
                // 45度模式直接开始
                g_targetAngle = 45.0f;
                ConfigureControlMode(g_workMode);
                g_systemState = STATE_RUNNING;
                g_modeStartTime = ANGLE_CONTROL_GetTime();
            }
            else if (g_workMode != MODE_IDLE)
            {
                g_systemState = STATE_ANGLE_SETTING;
            }
        }
        break;

    case STATE_ANGLE_SETTING:
        // 角度设置
        if (key == KEY_UP)
        {
            g_targetAngle += 5.0f;
            if (g_workMode == MODE_SINGLE_FAN_ANY && g_targetAngle > 90.0f)
                g_targetAngle = 90.0f;
            else if (g_workMode == MODE_DUAL_FAN_ANY && g_targetAngle > 180.0f)
                g_targetAngle = 180.0f;
        }
        else if (key == KEY_DOWN)
        {
            g_targetAngle -= 5.0f;
            if (g_targetAngle < 0.0f)
                g_targetAngle = 0.0f;
        }
        else if (key == KEY_ENTER)
        {
            // 开始控制
            ConfigureControlMode(g_workMode);
            ANGLE_CONTROL_SetTarget(&g_angle_control, g_targetAngle);
            g_systemState = STATE_RUNNING;
            g_modeStartTime = ANGLE_CONTROL_GetTime();
        }
        else if (key == KEY_MODE)
        {
            // 返回菜单
            g_systemState = STATE_MENU;
        }
        break;

    case STATE_RUNNING:
        if (key == KEY_MODE)
        {
            // 停止控制，返回菜单
            ANGLE_CONTROL_Stop(&g_angle_control);
            g_systemState = STATE_MENU;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief  配置控制模式
 * @param  mode: 工作模式
 * @retval 无
 */
static void ConfigureControlMode(WorkMode_TypeDef mode)
{
    switch (mode)
    {
    case MODE_SINGLE_FAN_45DEG:
        ANGLE_CONTROL_SetMode(&g_angle_control, CONTROL_MODE_SINGLE_FAN);
        ANGLE_CONTROL_SetStableCondition(&g_angle_control, 5.0f, 3000);
        ANGLE_CONTROL_SetTarget(&g_angle_control, 45.0f);
        break;

    case MODE_SINGLE_FAN_ANY:
        ANGLE_CONTROL_SetMode(&g_angle_control, CONTROL_MODE_SINGLE_FAN);
        ANGLE_CONTROL_SetStableCondition(&g_angle_control, 5.0f, 3000);
        break;

    case MODE_DUAL_FAN_ANY:
        ANGLE_CONTROL_SetMode(&g_angle_control, CONTROL_MODE_DUAL_FAN);
        ANGLE_CONTROL_SetStableCondition(&g_angle_control, 3.0f, 5000);
        break;

    case MODE_DUAL_FAN_SEQUENCE:
    {
        float angles[] = {45.0f, 60.0f, 90.0f, 120.0f, 135.0f};
        uint8_t times[] = {3, 3, 3, 3, 3};
        ANGLE_CONTROL_SetMode(&g_angle_control, CONTROL_MODE_SEQUENCE);
        ANGLE_CONTROL_ConfigSequence(&g_angle_control, angles, times, 5);
        ANGLE_CONTROL_SetStableCondition(&g_angle_control, 3.0f, 3000);
    }
    break;

    default:
        ANGLE_CONTROL_SetMode(&g_angle_control, CONTROL_MODE_IDLE);
        break;
    }
}

/**
 * @brief  显示状态更新函数
 * @param  无
 * @retval 无
 */
void DisplayStatus(void)
{
    static uint8_t first_run = 1;
    static uint32_t last_display_time = 0;
    uint32_t current_time = ANGLE_CONTROL_GetTime();
    char buf[4][21]; // 4行文本
    float current_angle = ANGLE_SENSOR_GetAngle();
    uint32_t elapsed = 0;
    uint8_t needUpdate = 0;
    uint8_t i;

    // 首次运行强制更新
    if (first_run)
    {
        first_run = 0;
        needUpdate = 1;
    }

    // 别的状态强制更新 - 这是关键，确保状态变化时立即更新显示
    if (g_systemState != g_lastSystemState || g_workMode != g_lastWorkMode)
    {
        needUpdate = 1;
    }

    // 检查目标角度变化 (在所有状态下) - 这是修复的关键点
    if (fabs(g_targetAngle - g_lastTargetAngle) > 0.2f)
    {
        needUpdate = 1;
    }

    // 限制刷新频率 - 只有不需要更新时才返回
    if (current_time - last_display_time < 100 && !needUpdate)
    {
        return;
    }

    // 清空临时缓冲区
    for (i = 0; i < 4; i++)
    {
        memset(buf[i], 0, sizeof(buf[i]));
    }

    // 检查当前角度变化 (仅在运行状态)
    if (g_systemState == STATE_RUNNING)
    {
        if (fabs(current_angle - g_lastCurrentAngle) > 0.5f)
        {
            needUpdate = 1;
        }
    }

    // 根据系统状态生成显示内容
    switch (g_systemState)
    {
    case STATE_MENU:
        // 标题栏
        strcpy(buf[0], "< Mode Selection >");

        // 当前选中的模式，使用指针标记
        switch (g_workMode)
        {
        case MODE_IDLE:
            strcpy(buf[1], "> Idle Mode");
            strcpy(buf[2], "  Single Fan 45");
            break;
        case MODE_SINGLE_FAN_45DEG:
            strcpy(buf[1], "  Idle Mode");
            strcpy(buf[2], "> Single Fan 45");
            break;
        case MODE_SINGLE_FAN_ANY:
            strcpy(buf[1], "> Single Fan Any");
            strcpy(buf[2], "  Dual Fan Any");
            break;
        case MODE_DUAL_FAN_ANY:
            strcpy(buf[1], "  Single Fan Any");
            strcpy(buf[2], "> Dual Fan Any");
            break;
        case MODE_DUAL_FAN_SEQUENCE:
            strcpy(buf[1], "> Sequence Mode");
            strcpy(buf[2], "  Idle Mode");
            break;
        }

        // 底部提示
        strcpy(buf[3], "UP/DN:Sel ENTER:OK");
        break;

    case STATE_ANGLE_SETTING:
        strcpy(buf[0], "< Angle Setting >");
        sprintf(buf[1], "Target: %.1f deg", g_targetAngle);
        sprintf(buf[2], "Current: %.1f deg", current_angle);
        strcpy(buf[3], "UP/DN:+- ENTER:OK");
        break;

    case STATE_RUNNING:
        // 显示运行状态
        switch (g_workMode)
        {
        case MODE_SINGLE_FAN_45DEG:
            strcpy(buf[0], "< Single Fan 45 >");
            break;
        case MODE_SINGLE_FAN_ANY:
            strcpy(buf[0], "< Single Fan >");
            break;
        case MODE_DUAL_FAN_ANY:
            strcpy(buf[0], "< Dual Fan >");
            break;
        case MODE_DUAL_FAN_SEQUENCE:
            strcpy(buf[0], "< Sequence Mode >");
            break;
        default:
            strcpy(buf[0], "< Running >");
            break;
        }

        // 显示角度信息
        sprintf(buf[1], "Target: %.1f deg", g_targetAngle);
        sprintf(buf[2], "Current: %.1f deg", current_angle);

        // 模式特定信息
        if (g_workMode == MODE_SINGLE_FAN_45DEG)
        {
            elapsed = (current_time - g_modeStartTime) / 1000;
            if (elapsed <= 10)
            {
                sprintf(buf[3], "Time: %ds/10s", (int)elapsed);
                if (elapsed != g_lastElapsedTime)
                {
                    needUpdate = 1;
                }
            }
            else
            {
                strcpy(buf[3], "Time: Complete");
            }
        }
        else if (g_workMode == MODE_DUAL_FAN_SEQUENCE)
        {
            // 序列模式特殊信息显示
            sprintf(buf[3], "Step: %d/5", g_angle_control.sequence.current_index + 1);
        }
        else
        {
            strcpy(buf[3], "MODE:Back");
        }
        break;

    default:
        strcpy(buf[0], "< System Error >");
        strcpy(buf[1], "Unknown state");
        sprintf(buf[2], "State: %d", g_systemState);
        strcpy(buf[3], "Press MODE to reset");
        break;
    }

    // 检查缓冲区内容是否变化
    if (!needUpdate)
    {
        for (i = 0; i < 4; i++)
        {
            if (strcmp(buf[i], g_lastDisplayBuf[i]) != 0)
            {
                needUpdate = 1;
                break;
            }
        }
    }

    // 更新显示
    if (needUpdate)
    {
        OLED_Clear();

        // 显示生成的内容
        for (i = 0; i < 4; i++)
        {
            if (buf[i][0] != '\0')
            {
                OLED_ShowString(0, i * 16, (u8 *)buf[i], 12, 1);
            }
        }

        // 保存当前显示内容
        for (i = 0; i < 4; i++)
        {
            strncpy(g_lastDisplayBuf[i], buf[i], 20);
            g_lastDisplayBuf[i][20] = '\0';
        }

        // 更新状态变量
        g_lastSystemState = g_systemState;
        g_lastWorkMode = g_workMode;
        g_lastCurrentAngle = current_angle;
        g_lastTargetAngle = g_targetAngle; // 确保更新目标角度记录值
        g_lastElapsedTime = elapsed;

        // 刷新显示
        OLED_Refresh();
        last_display_time = current_time;
    }
}

/**
 * @brief  菜单管理函数
 * @param  无
 * @retval 无
 */
static void MenuManager(void)
{
    // 根据系统状态管理菜单
    // 此处可以添加菜单管理逻辑
    return;
}
