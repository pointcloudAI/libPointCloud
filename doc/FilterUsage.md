# Filter Usage for libPointCloud SDK

1. DarkPixFilter
2. DenoiseFilter
3. FlypixFilter
4. HDRFilter
5. IIRFilter
6. MedianFilter
7. TemporalMedianFilter
8. SmoothFilter
9. BilateralFilter
    
## DarkPixFilter

| Parameter  | Desc | Min  | Max  |
|-------|:---:|-----------|-------:|
| aThrNear  | Amp Near Threshold | 0.0f      | 4095.0f |
| phThrNear  | Phase Near Threshold | 0.0f      | 4095.0f |
| aThrFar  | Amp Far Threshold | 0.0f      | 4095.0f |
| phThrFar  | Phase Far Threshold | 0.0f      | 4095.0f |
| ambThresh  | Ambient Threshold | 0.0f      | 4095.0f |

## DenoiseFilter

| Parameter  | Desc | Min  | Max  |
|-------|:---:|-----------|-------:|
| order  | Order of the filter | 2     | 100 |
| threshold  | Threshold | 0     | 10000 |

## FlypixFilter

| Parameter  | Desc | Min  | Max  |
|-------|:---:|-----------|-------:|
| threshold  | Gradient Threshold | 0     | 10000 |

## HDRFilter

| Parameter  | Desc | Min  | Max  |
|-------|:---:|-----------|-------:|
| order  | Order of the filter | 2     | 100 |

## IIRFilter   

| Parameter  | Desc | Min  | Max  |
|-------|:---:|-----------|-------:|
| gain  | IIR gain coefficient | 0.0f     | 1.0f |

## MedianFilter   

| Parameter  | Desc | Min  | Max  |
|-------|:---:|-----------|-------:|
| stability  | Stability factor | 0.0f      | 1.0f |
| deadband  | Dead band | 0.0f      | 1.0f |
| deadbandStep  | Dead band step | 0.0f      | 1.0f |
| halfKernelSize  | Half kernel size | 1      | 100|
| ambThresh  | Ambient Threshold | 0.0f      | 4095.0f |

## TemporalMedianFilter   

| Parameter  | Desc | Min  | Max  |
|-------|:---:|-----------|-------:|
| order  | Order of the filter | 1      | 100 |
| deadband  | Dead band | 0.0f      | 1.0f |

## TemporalMedianFilter   

| Parameter  | Desc | Min  | Max  |
|-------|:---:|-----------|-------:|
| sigma  | Standard deviation | 1      | 100 |
| deadband  | Dead band | 0.0f      | 1.0f |

## SmoothFilter


| Parameter  | Desc | Min  | Max  |
|-------|:---:|-----------|-------:|
| sigma  | Standard deviation | 1      | 100 |

## BilateralFilter

| Parameter  | Desc | Min  | Max  |
|-------|:---:|-----------|-------:|
| sigma  | Standard deviation | 1      | 100 |
