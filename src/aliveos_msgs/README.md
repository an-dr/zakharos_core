# aliveos_msgs

## Json schemas

Schemas's location: [json](json)

### Example: Command Concept

```json
{
    "$schema": "/home/agramakov/Files/Code/zakharos_core/src/aliveos_msgs/json/command-concept-dsc.json",
    "name": "move forward",
    "descriptor": [
        {
            "modifier": "",
            "commands": [
                {
                    "device_name": "motors",
                    "command": "move",
                    "arguments": [
                        "forward",
                        "100ms"
                    ]
                }
            ]
        },
        {
            "modifier": "long",
            "commands": [
                {
                    "device_name": "motors",
                    "command": "move",
                    "arguments": [
                        "forward",
                        "1000ms"
                    ]
                }
            ]
        }
    ]
}
```

### Example: Emotion Core Animal Parameters

```json
{
    "$schema": "../src/schemas/emotion-core-animal-params-1.0.json",
    "adrenaline": 0,
    "cortisol": 11,
    "dopamine": 22.2,
    "melatonin": 33.33,
    "oxytocin": 444.44,
    "serotonin": 555.555
}
```

### Example: Emotion Core Animal Weights

```json
{
    "$schema": "../src/schemas/emotion-core-animal-weights-1.0.json",
    "weights": [
        { "parameter": "adrenaline", "value": -0.1 },
        { "parameter": "dopamine", "value": 2 }
    ]
}
```

### Example: Emotion Core Generic Weights

```json
{
    "$schema": "../src/schemas/emotion-core-generic-weights-1.0.json",
    "weights": [
        { "parameter": "param1", "value": -0.1 },
        { "parameter": "param2", "value": 10 },
        { "parameter": "param3", "value": -100 }
    ]
}
```

### Example: Perception Concept Descriptor

```json
{
    "$schema": "perception-concept-descriptor.json",
    "data_types": [
        {
            "data_type": "value",
            "descriptor": [
                {
                    "concept": "temperature_cold",
                    "conditions": [ { "condition": "LESS_THAN_OR_EQUAL", "threshold": 10 } ]
                },
                {
                    "concept": "temperature_ok",
                    "conditions": [ { "condition": "GREATER_THAN", "threshold": 10 },
                                    { "condition": "LESS_THAN", "threshold": 30 } ]
                },
                {
                    "concept": "temperature_bloody_hell",
                    "conditions": [ { "condition": "GREATER_THAN", "threshold": 1000000 } ]
                }
            ]
        }
    ]
}
```

### Example: Emotion Core Data Descriptor

```json
{
    "$schema": "perception-concept-descriptor.json",
    "data_type": "temperature",
    "value_min": 0,
    "value_max": 100,
    "weights": [
        { "parameter": "adrenaline", "value": -0.1 },
        { "parameter": "dopamine", "value": 2 }
    ]
}
```
