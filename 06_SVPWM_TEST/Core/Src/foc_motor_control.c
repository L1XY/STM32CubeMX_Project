#include "foc_motor_control.h"

float arm_sin(float x);
float arm_cos(float x);

/********************************************************************************
 * Clarke变换（3相->2相）
 *********************************************************************************/
FOC_Alpha_Beta_t FOC_Clarke_Transform(const FOC_U_V_W_t *i_uvw)
{
    FOC_Alpha_Beta_t i_AlphaBeta;
#if 1
    /**采用等幅值变换
     * | α | = |  1       0   | | u |
     * | β | = | 1/√3    2/√3 | | V |
     */
    i_AlphaBeta.alpha = i_uvw->iu;
    i_AlphaBeta.beta = ((i_uvw->iu + 2.0f * i_uvw->iv) * _1_DIV_SQRT_3);
#else
    /**没有采用等幅值变换
     * | α | = | 3/2    0  | | u |
     * | β | = | √3/2   √3 | | V |
     */
    i_AlphaBeta.alpha = 1.5f * i_uvw->iu;
    i_AlphaBeta.beta = SQRT_3_DIV_2 * i_uvw->iu + SQRT_3 * i_uvw->iv;
#endif

    return i_AlphaBeta;
}

/********************************************************************************
 * Park变换（静止坐标系->旋转坐标系）
 *********************************************************************************/
FOC_D_Q_t FOC_Park_Transform(const FOC_Alpha_Beta_t *i_AlphaBeta, const float ElectricalAngle)
{
    FOC_D_Q_t i_DQ;
    float sinTheta = arm_sin(ElectricalAngle);
    float cosTheta = arm_cos(ElectricalAngle);

    /**
     * | Id | = | cosθ    sinθ | | α |
     * | Iq | = | -sinθ   cosθ | | β |
     */
    i_DQ.id = cosTheta * i_AlphaBeta->alpha + sinTheta * i_AlphaBeta->beta;
    i_DQ.iq = (-sinTheta) * i_AlphaBeta->alpha + cosTheta * i_AlphaBeta->beta;

    return i_DQ;
}

/********************************************************************************
 * Park逆变换（旋转坐标系-> α β 坐标系）
 *********************************************************************************/
FOC_Alpha_Beta_t FOC_Inverse_Park_Transform(const FOC_D_Q_t *i_DQ, const float ElectricalAngle)
{
    FOC_Alpha_Beta_t i_AlphaBeta;

    float cosTheta = arm_cos(ElectricalAngle);
    float sinTheta = arm_sin(ElectricalAngle);
    /**
     * | Iα | = | cosθ   -sinθ | | Id |
     * | Iβ | = | sinθ    cosθ | | Iq |
     */
    i_AlphaBeta.alpha = cosTheta * i_DQ->id - sinTheta * i_DQ->iq;
    i_AlphaBeta.beta = sinTheta * i_DQ->id + cosTheta * i_DQ->iq;

    return i_AlphaBeta;
}

/********************************************************************************
 * Clarke逆变换（2相->3相）
 *********************************************************************************/
FOC_U_V_W_t FOC_Inverse_Clarke_Transform(const FOC_Alpha_Beta_t *i_AlphaBeta)
{
    FOC_U_V_W_t i_UVW;
    /**
     * | Iu | = |   1      0   | | Iα |
     * | Iv | = | -1/2    √3/2 | |    |
     * | Iw | = | -1/2   -√3/2 | | Iβ |
     */
    i_UVW.iu = i_AlphaBeta->alpha;
    i_UVW.iv = (-i_AlphaBeta->alpha + SQRT_3 * (i_AlphaBeta->beta)) / 2.0f;
    i_UVW.iw = (-i_AlphaBeta->alpha - SQRT_3 * (i_AlphaBeta->beta)) / 2.0f;

    return i_UVW;
}

/********************************************************************************
 * 电角度限幅（0 ~ 2π）
 *********************************************************************************/
void Limit_Angle(float *ElectricalAngle)
{
    while (*ElectricalAngle > _2PI || *ElectricalAngle < 0)
    {
        if (*ElectricalAngle > _2PI)
        {
            *ElectricalAngle -= _2PI;
        }
        else if (*ElectricalAngle < 0)
        {
            *ElectricalAngle += _2PI;
        }
    }
}

static float test_ElectricalAngle = 0;
FOC_U_V_W_t I_uvw;
FOC_Alpha_Beta_t I_AlphaBeta;
FOC_D_Q_t I_dq;

void FOC_ClarkePark_Debug(void)
{
    test_ElectricalAngle += 0.1f;
    Limit_Angle(&test_ElectricalAngle); /* 模拟生成三相正弦波 */
    I_uvw.iu = arm_sin(test_ElectricalAngle);
    I_uvw.iv = arm_sin(test_ElectricalAngle + _PI_3 * 2.0f);
    I_uvw.iw = arm_sin(test_ElectricalAngle + _PI_3 * 2.0f + _PI_3 * 2.0f);
    I_AlphaBeta = FOC_Clarke_Transform(&I_uvw);
    I_dq = FOC_Park_Transform(&I_AlphaBeta, atan2f(I_AlphaBeta.beta, I_AlphaBeta.alpha));
    debug("%f,%f,%f,%f,%f,%f,%f,%f\r\n",
          test_ElectricalAngle,
          I_uvw.iu,
          I_uvw.iv,
          I_uvw.iw,
          I_AlphaBeta.alpha,
          I_AlphaBeta.beta,
          I_dq.id,
          I_dq.iq);
}

void FOC_InverseParkInverseClarke_Debug(void)
{
    test_ElectricalAngle += 0.1f;
    I_dq.id = 0.0;
    I_dq.iq = 0.5;
    Limit_Angle(&test_ElectricalAngle);
    I_AlphaBeta = FOC_Inverse_Park_Transform(&I_dq, test_ElectricalAngle); /* 帕克逆变换 */
    I_uvw = FOC_Inverse_Clarke_Transform(&I_AlphaBeta);                    /* 克拉克逆变换 */
    debug("%f,%f,%f,%f,%f,%f,%f,%f\r\n",
          test_ElectricalAngle,
          I_uvw.iu,
          I_uvw.iv,
          I_uvw.iw,
          I_AlphaBeta.alpha,
          I_AlphaBeta.beta,
          I_dq.id,
          I_dq.iq);
}

uint8_t FOC_SVPWM_GetSector(const FOC_Alpha_Beta_t *I_AlphaBeta)
{
    uint8_t sector = 0;

    if (I_AlphaBeta->beta > 0.0f)
    {
        sector = 1;
    }
    if ((SQRT_3 * I_AlphaBeta->alpha - I_AlphaBeta->beta) / 2.0f > 0.0f)
    {
        sector += 2;
    }
    if ((-SQRT_3 * I_AlphaBeta->alpha - I_AlphaBeta->beta) / 2.0f > 0.0f)
    {
        sector += 4;
    }
#if 0
    float temp_u_Beta;
    temp_u_Beta = I_AlphaBeta->beta * _1_DIV_SQRT_3;

    if (I_AlphaBeta->alpha > 0)
    {
        if (I_AlphaBeta->beta > 0)
        {
            if (temp_u_Beta > I_AlphaBeta->alpha)
            {
                sector = 1;
            }
            else
            {
                sector = 0;
            }
        }
        else
        {
            if (temp_u_Beta < (-(I_AlphaBeta->alpha)))
            {
                sector = 4;
            }
            else
            {
                sector = 5;
            }
        }
    }
    else
    {
        if (I_AlphaBeta->beta > 0)
        {
            if (temp_u_Beta > (-(I_AlphaBeta->alpha)))
            {
                sector = 1;
            }
            else
            {
                sector = 2;
            }
        }
        else
        {
            if (temp_u_Beta < I_AlphaBeta->alpha)
            {
                sector = 4;
            }
            else
            {
                sector = 3;
            }
        }
    }
#endif
    return sector;
}

const float sinVector[7] =
    {
        0,
        1.5,
        1.5,
        0,
        -1.5,
        -1.5,
        0,
};
const float cosVector[7] =
    {
        SQRT_3,
        SQRT_3_DIV_2,
        -SQRT_3_DIV_2,
        -SQRT_3,
        -SQRT_3_DIV_2,
        SQRT_3_DIV_2,
        SQRT_3,
};
FOC_VectorTime_t FOC_SVPWM_GetVectorTime(uint8_t sector, FOC_Alpha_Beta_t *I_AlphaBeta)
{
    FOC_VectorTime_t t_VectorTime;
    float Tx, Ty, f_temp;
    const float Ts = 1.0f;
    const float TsDivUdc = Ts / UDC;
    float Ualpha = I_AlphaBeta->alpha;
    float Ubeta = I_AlphaBeta->beta;

    switch (sector)
    {
    case 1:
        Tx = (-1.5f * Ualpha + SQRT_3_DIV_2 * Ubeta) * TsDivUdc;
        Ty = (1.5f * Ualpha + SQRT_3_DIV_2 * Ubeta) * TsDivUdc;
        break;

    case 2:
        Tx = (1.5f * Ualpha + SQRT_3_DIV_2 * Ubeta) * TsDivUdc;
        Ty = (-SQRT_3 * Ubeta) * TsDivUdc;
        break;

    case 3:
        Tx = (1.5f * Ualpha - SQRT_3_DIV_2 * Ubeta) * TsDivUdc;
        Ty = (SQRT_3 * Ubeta) * TsDivUdc;
        break;

    case 4:
        Tx = (-SQRT_3 * Ubeta) * TsDivUdc;
        Ty = (-1.5f * Ualpha + SQRT_3_DIV_2 * Ubeta) * TsDivUdc;
        break;

    case 5:
        Tx = SQRT_3 * Ubeta * TsDivUdc;
        Ty = (-1.5f * Ualpha - SQRT_3_DIV_2 * Ubeta) * TsDivUdc;
        break;

    default:
        Tx = (-1.5f * Ualpha - SQRT_3_DIV_2 * Ubeta) * TsDivUdc;
        Ty = (1.5f * Ualpha - SQRT_3_DIV_2 * Ubeta) * TsDivUdc;
        break;
    }
    f_temp = Tx + Ty;
    if (f_temp > Ts)
    {
        Tx /= f_temp;
        Ty /= f_temp;
    }
    t_VectorTime.t0 = (Ts - (Tx + Ty)) / 4.0f;
    t_VectorTime.t1 = Tx / 2.0f + t_VectorTime.t0;
    t_VectorTime.t2 = Ty / 2.0f + t_VectorTime.t1;

    return t_VectorTime;
}

FOC_PWMCounter_t FOC_SVPWM_GetPWMCounter(uint8_t sector, const FOC_VectorTime_t *t_VectorTime)
{
    FOC_PWMCounter_t c_PWMCounter;
    switch (sector)
    {
    case 1:
        c_PWMCounter.counter_0 = t_VectorTime->t1;
        c_PWMCounter.counter_1 = t_VectorTime->t0;
        c_PWMCounter.counter_2 = t_VectorTime->t2;
        break;

    case 2:
        c_PWMCounter.counter_0 = t_VectorTime->t0;
        c_PWMCounter.counter_1 = t_VectorTime->t2;
        c_PWMCounter.counter_2 = t_VectorTime->t1;
        break;

    case 3:
        c_PWMCounter.counter_0 = t_VectorTime->t0;
        c_PWMCounter.counter_1 = t_VectorTime->t1;
        c_PWMCounter.counter_2 = t_VectorTime->t2;
        break;

    case 4:
        c_PWMCounter.counter_0 = t_VectorTime->t2;
        c_PWMCounter.counter_1 = t_VectorTime->t1;
        c_PWMCounter.counter_2 = t_VectorTime->t0;
        break;

    case 5:
        c_PWMCounter.counter_0 = t_VectorTime->t2;
        c_PWMCounter.counter_1 = t_VectorTime->t0;
        c_PWMCounter.counter_2 = t_VectorTime->t1;
        break;

    case 6:
        c_PWMCounter.counter_0 = t_VectorTime->t1;
        c_PWMCounter.counter_1 = t_VectorTime->t2;
        c_PWMCounter.counter_2 = t_VectorTime->t0;
        break;
    }
    c_PWMCounter.counter_0 *= 5000;
    c_PWMCounter.counter_1 *= 5000;
    c_PWMCounter.counter_2 *= 5000;

    return c_PWMCounter;
}
void FOC_SVPWM_Debug(void)
{
    uint8_t sector;
    FOC_VectorTime_t t_VectorTime;
    FOC_PWMCounter_t c_PWMCounter;
    test_ElectricalAngle += 0.1f;
    I_dq.id = 0.0;
    I_dq.iq = 2.5;
    Limit_Angle(&test_ElectricalAngle);
    I_AlphaBeta = FOC_Inverse_Park_Transform(&I_dq, test_ElectricalAngle); /* 帕克逆变换 */
    sector = FOC_SVPWM_GetSector(&I_AlphaBeta);
    t_VectorTime = FOC_SVPWM_GetVectorTime(sector, &I_AlphaBeta);
    c_PWMCounter = FOC_SVPWM_GetPWMCounter(sector, &t_VectorTime);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, c_PWMCounter.counter_0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, c_PWMCounter.counter_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, c_PWMCounter.counter_2);
    debug("%f,%f,%f,%f,%f,%f,%f,%f\r\n",
          test_ElectricalAngle,
            /*
            t_VectorTime.t0*10, \
            t_VectorTime.t1*10, \
            t_VectorTime.t2*10, \
            */
          c_PWMCounter.counter_0,
          c_PWMCounter.counter_1,
          c_PWMCounter.counter_2,
          I_AlphaBeta.alpha,
          I_AlphaBeta.beta,
          I_dq.id,
          I_dq.iq);
}

const float sinTable[512 + 1] =
    {
    0.00000000f, 0.01227154f, 0.02454123f, 0.03680722f, 0.04906767f, 0.06132074f,
    0.07356456f, 0.08579731f, 0.09801714f, 0.11022221f, 0.12241068f, 0.13458071f,
    0.14673047f, 0.15885814f, 0.17096189f, 0.18303989f, 0.19509032f, 0.20711138f,
    0.21910124f, 0.23105811f, 0.24298018f, 0.25486566f, 0.26671276f, 0.27851969f,
    0.29028468f, 0.30200595f, 0.31368174f, 0.32531029f, 0.33688985f, 0.34841868f,
    0.35989504f, 0.37131719f, 0.38268343f, 0.39399204f, 0.40524131f, 0.41642956f,
    0.42755509f, 0.43861624f, 0.44961133f, 0.46053871f, 0.47139674f, 0.48218377f,
    0.49289819f, 0.50353838f, 0.51410274f, 0.52458968f, 0.53499762f, 0.54532499f,
    0.55557023f, 0.56573181f, 0.57580819f, 0.58579786f, 0.59569930f, 0.60551104f,
    0.61523159f, 0.62485949f, 0.63439328f, 0.64383154f, 0.65317284f, 0.66241578f,
    0.67155895f, 0.68060100f, 0.68954054f, 0.69837625f, 0.70710678f, 0.71573083f,
    0.72424708f, 0.73265427f, 0.74095113f, 0.74913639f, 0.75720885f, 0.76516727f,
    0.77301045f, 0.78073723f, 0.78834643f, 0.79583690f, 0.80320753f, 0.81045720f,
    0.81758481f, 0.82458930f, 0.83146961f, 0.83822471f, 0.84485357f, 0.85135519f,
    0.85772861f, 0.86397286f, 0.87008699f, 0.87607009f, 0.88192126f, 0.88763962f,
    0.89322430f, 0.89867447f, 0.90398929f, 0.90916798f, 0.91420976f, 0.91911385f,
    0.92387953f, 0.92850608f, 0.93299280f, 0.93733901f, 0.94154407f, 0.94560733f,
    0.94952818f, 0.95330604f, 0.95694034f, 0.96043052f, 0.96377607f, 0.96697647f,
    0.97003125f, 0.97293995f, 0.97570213f, 0.97831737f, 0.98078528f, 0.98310549f,
    0.98527764f, 0.98730142f, 0.98917651f, 0.99090264f, 0.99247953f, 0.99390697f,
    0.99518473f, 0.99631261f, 0.99729046f, 0.99811811f, 0.99879546f, 0.99932238f,
    0.99969882f, 0.99992470f, 1.00000000f, 0.99992470f, 0.99969882f, 0.99932238f,
    0.99879546f, 0.99811811f, 0.99729046f, 0.99631261f, 0.99518473f, 0.99390697f,
    0.99247953f, 0.99090264f, 0.98917651f, 0.98730142f, 0.98527764f, 0.98310549f,
    0.98078528f, 0.97831737f, 0.97570213f, 0.97293995f, 0.97003125f, 0.96697647f,
    0.96377607f, 0.96043052f, 0.95694034f, 0.95330604f, 0.94952818f, 0.94560733f,
    0.94154407f, 0.93733901f, 0.93299280f, 0.92850608f, 0.92387953f, 0.91911385f,
    0.91420976f, 0.90916798f, 0.90398929f, 0.89867447f, 0.89322430f, 0.88763962f,
    0.88192126f, 0.87607009f, 0.87008699f, 0.86397286f, 0.85772861f, 0.85135519f,
    0.84485357f, 0.83822471f, 0.83146961f, 0.82458930f, 0.81758481f, 0.81045720f,
    0.80320753f, 0.79583690f, 0.78834643f, 0.78073723f, 0.77301045f, 0.76516727f,
    0.75720885f, 0.74913639f, 0.74095113f, 0.73265427f, 0.72424708f, 0.71573083f,
    0.70710678f, 0.69837625f, 0.68954054f, 0.68060100f, 0.67155895f, 0.66241578f,
    0.65317284f, 0.64383154f, 0.63439328f, 0.62485949f, 0.61523159f, 0.60551104f,
    0.59569930f, 0.58579786f, 0.57580819f, 0.56573181f, 0.55557023f, 0.54532499f,
    0.53499762f, 0.52458968f, 0.51410274f, 0.50353838f, 0.49289819f, 0.48218377f,
    0.47139674f, 0.46053871f, 0.44961133f, 0.43861624f, 0.42755509f, 0.41642956f,
    0.40524131f, 0.39399204f, 0.38268343f, 0.37131719f, 0.35989504f, 0.34841868f,
    0.33688985f, 0.32531029f, 0.31368174f, 0.30200595f, 0.29028468f, 0.27851969f,
    0.26671276f, 0.25486566f, 0.24298018f, 0.23105811f, 0.21910124f, 0.20711138f,
    0.19509032f, 0.18303989f, 0.17096189f, 0.15885814f, 0.14673047f, 0.13458071f,
    0.12241068f, 0.11022221f, 0.09801714f, 0.08579731f, 0.07356456f, 0.06132074f,
    0.04906767f, 0.03680722f, 0.02454123f, 0.01227154f, 0.00000000f, -0.01227154f,
    -0.02454123f, -0.03680722f, -0.04906767f, -0.06132074f, -0.07356456f,
    -0.08579731f, -0.09801714f, -0.11022221f, -0.12241068f, -0.13458071f,
    -0.14673047f, -0.15885814f, -0.17096189f, -0.18303989f, -0.19509032f,
    -0.20711138f, -0.21910124f, -0.23105811f, -0.24298018f, -0.25486566f,
    -0.26671276f, -0.27851969f, -0.29028468f, -0.30200595f, -0.31368174f,
    -0.32531029f, -0.33688985f, -0.34841868f, -0.35989504f, -0.37131719f,
    -0.38268343f, -0.39399204f, -0.40524131f, -0.41642956f, -0.42755509f,
    -0.43861624f, -0.44961133f, -0.46053871f, -0.47139674f, -0.48218377f,
    -0.49289819f, -0.50353838f, -0.51410274f, -0.52458968f, -0.53499762f,
    -0.54532499f, -0.55557023f, -0.56573181f, -0.57580819f, -0.58579786f,
    -0.59569930f, -0.60551104f, -0.61523159f, -0.62485949f, -0.63439328f,
    -0.64383154f, -0.65317284f, -0.66241578f, -0.67155895f, -0.68060100f,
    -0.68954054f, -0.69837625f, -0.70710678f, -0.71573083f, -0.72424708f,
    -0.73265427f, -0.74095113f, -0.74913639f, -0.75720885f, -0.76516727f,
    -0.77301045f, -0.78073723f, -0.78834643f, -0.79583690f, -0.80320753f,
    -0.81045720f, -0.81758481f, -0.82458930f, -0.83146961f, -0.83822471f,
    -0.84485357f, -0.85135519f, -0.85772861f, -0.86397286f, -0.87008699f,
    -0.87607009f, -0.88192126f, -0.88763962f, -0.89322430f, -0.89867447f,
    -0.90398929f, -0.90916798f, -0.91420976f, -0.91911385f, -0.92387953f,
    -0.92850608f, -0.93299280f, -0.93733901f, -0.94154407f, -0.94560733f,
    -0.94952818f, -0.95330604f, -0.95694034f, -0.96043052f, -0.96377607f,
    -0.96697647f, -0.97003125f, -0.97293995f, -0.97570213f, -0.97831737f,
    -0.98078528f, -0.98310549f, -0.98527764f, -0.98730142f, -0.98917651f,
    -0.99090264f, -0.99247953f, -0.99390697f, -0.99518473f, -0.99631261f,
    -0.99729046f, -0.99811811f, -0.99879546f, -0.99932238f, -0.99969882f,
    -0.99992470f, -1.00000000f, -0.99992470f, -0.99969882f, -0.99932238f,
    -0.99879546f, -0.99811811f, -0.99729046f, -0.99631261f, -0.99518473f,
    -0.99390697f, -0.99247953f, -0.99090264f, -0.98917651f, -0.98730142f,
    -0.98527764f, -0.98310549f, -0.98078528f, -0.97831737f, -0.97570213f,
    -0.97293995f, -0.97003125f, -0.96697647f, -0.96377607f, -0.96043052f,
    -0.95694034f, -0.95330604f, -0.94952818f, -0.94560733f, -0.94154407f,
    -0.93733901f, -0.93299280f, -0.92850608f, -0.92387953f, -0.91911385f,
    -0.91420976f, -0.90916798f, -0.90398929f, -0.89867447f, -0.89322430f,
    -0.88763962f, -0.88192126f, -0.87607009f, -0.87008699f, -0.86397286f,
    -0.85772861f, -0.85135519f, -0.84485357f, -0.83822471f, -0.83146961f,
    -0.82458930f, -0.81758481f, -0.81045720f, -0.80320753f, -0.79583690f,
    -0.78834643f, -0.78073723f, -0.77301045f, -0.76516727f, -0.75720885f,
    -0.74913639f, -0.74095113f, -0.73265427f, -0.72424708f, -0.71573083f,
    -0.70710678f, -0.69837625f, -0.68954054f, -0.68060100f, -0.67155895f,
    -0.66241578f, -0.65317284f, -0.64383154f, -0.63439328f, -0.62485949f,
    -0.61523159f, -0.60551104f, -0.59569930f, -0.58579786f, -0.57580819f,
    -0.56573181f, -0.55557023f, -0.54532499f, -0.53499762f, -0.52458968f,
    -0.51410274f, -0.50353838f, -0.49289819f, -0.48218377f, -0.47139674f,
    -0.46053871f, -0.44961133f, -0.43861624f, -0.42755509f, -0.41642956f,
    -0.40524131f, -0.39399204f, -0.38268343f, -0.37131719f, -0.35989504f,
    -0.34841868f, -0.33688985f, -0.32531029f, -0.31368174f, -0.30200595f,
    -0.29028468f, -0.27851969f, -0.26671276f, -0.25486566f, -0.24298018f,
    -0.23105811f, -0.21910124f, -0.20711138f, -0.19509032f, -0.18303989f,
    -0.17096189f, -0.15885814f, -0.14673047f, -0.13458071f, -0.12241068f,
    -0.11022221f, -0.09801714f, -0.08579731f, -0.07356456f, -0.06132074f,
    -0.04906767f, -0.03680722f, -0.02454123f, -0.01227154f, -0.00000000f,
};

float arm_cos(float x)
{
    float cosVal, fract, in; /* Temporary input, output variables */
    uint16_t index;          /* Index variable */
    float a, b;              /* Two nearest output values */
    int32_t n;
    float findex;

    /* input x is in radians */
    /* Scale input to [0 1] range from [0 2*PI] , divide input by 2*pi, add 0.25 (pi/2) to read sine table */
    in = x * 0.159154943092f + 0.25f;

    /* Calculation of floor value of input */
    n = (int32_t)in;

    /* Make negative values towards -infinity */
    if (in < 0.0f)
    {
        n--;
    }

    /* Map input value to [0 1] */
    in = in - (float)n;

    /* Calculation of index of the table */
    findex = (float)512 * in;
    index = (uint16_t)findex;

    /* when "in" is exactly 1, we need to rotate the index down to 0 */
    if (index >= 512)
    {
        index = 0;
        findex -= (float)512;
    }

    /* fractional value calculation */
    fract = findex - (float)index;

    /* Read two nearest values of input value from the cos table */
    a = sinTable[index];
    b = sinTable[index + 1];

    /* Linear interpolation process */
    cosVal = (1.0f - fract) * a + fract * b;

    /* Return output value */
    return (cosVal);
}

float arm_sin(float x)
{
    float sinVal, fract, in; /* Temporary input, output variables */
    uint16_t index;          /* Index variable */
    float a, b;              /* Two nearest output values */
    int32_t n;
    float findex;

    /* input x is in radians */
    /* Scale input to [0 1] range from [0 2*PI] , divide input by 2*pi */
    in = x * 0.159154943092f;

    /* Calculation of floor value of input */
    n = (int32_t)in;

    /* Make negative values towards -infinity */
    if (in < 0.0f)
    {
        n--;
    }

    /* Map input value to [0 1] */
    in = in - (float)n;

    /* Calculation of index of the table */
    findex = (float)512 * in;
    index = (uint16_t)findex;

    /* when "in" is exactly 1, we need to rotate the index down to 0 */
    if (index >= 512)
    {
        index = 0;
        findex -= (float)512;
    }

    /* fractional value calculation */
    fract = findex - (float)index;

    /* Read two nearest values of input value from the sin table */
    a = sinTable[index];
    b = sinTable[index + 1];

    /* Linear interpolation process */
    sinVal = (1.0f - fract) * a + fract * b;

    /* Return output value */
    return (sinVal);
}
