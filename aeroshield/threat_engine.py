def classify(score):
    if score >= 80:
        return "CRITICAL"
    if score >= 70:
        return "HIGH RISK"
    if score >= 60:
        return "SUSPICIOUS"
    return "NORMAL"

def decide(score, confirmed, movement, disagreement):
    level = classify(score)
    if level == "CRITICAL":
        action = "PRIORITY TRACK / IMMEDIATE REVIEW"
    elif level == "HIGH RISK":
        action = "PRIORITY TRACK / CONTINUOUS MONITORING"
    elif level == "SUSPICIOUS":
        action = "ENHANCED MONITORING"
    else:
        action = "ROUTINE MONITORING"

    if disagreement:
        evidence = "SENSOR DISAGREEMENT"
    elif confirmed:
        evidence = "MULTI-SENSOR CONFIRMATION"
    else:
        evidence = "PARTIAL SENSOR CORROBORATION"

    return {
        "level": level,
        "action": action,
        "evidence": evidence,
        "motion": movement,
    }
