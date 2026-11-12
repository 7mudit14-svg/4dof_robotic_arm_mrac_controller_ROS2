# Evaluation loop results

Error is plant position minus reference model, in rad.

## Mean |e|

| Evaluation | Joint 1 | Joint 2 | Joint 3 | Joint 4 |
|---|---|---|---|---|
| before training | 0.0018 | 0.0032 | 0.0020 | 0.0009 |
| after 50 training poses | 0.0023 | 0.0015 | 0.0020 | 0.0005 |
| after 200 training poses | 0.0017 | 0.0014 | 0.0015 | 0.0012 |

## Max |e|

| Evaluation | Joint 1 | Joint 2 | Joint 3 | Joint 4 |
|---|---|---|---|---|
| before training | 0.0091 | 0.0178 | 0.0094 | 0.0028 |
| after 50 training poses | 0.0079 | 0.0055 | 0.0089 | 0.0059 |
| after 200 training poses | 0.0094 | 0.0063 | 0.0057 | 0.0043 |

## Final e (end of the loop)

| Evaluation | Joint 1 | Joint 2 | Joint 3 | Joint 4 |
|---|---|---|---|---|
| before training | +0.0001 | -0.0007 | -0.0001 | +0.0000 |
| after 50 training poses | +0.0001 | -0.0018 | -0.0003 | +0.0000 |
| after 200 training poses | +0.0001 | -0.0006 | -0.0003 | +0.0000 |

## Training

200 random poses, 29.0 min of simulation time.
