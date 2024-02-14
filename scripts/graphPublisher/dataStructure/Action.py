
class Action:
    @staticmethod
    def release(A, S, O):
        sequence = [f"[ADD]{A}|hasHandMovingToward|{S}", f"[DEL]{A}|isHolding|{O}"]
        return sequence

    @staticmethod
    def grasp(A, C):
        sequence = [f"[ADD]{A}|hasHandMovingToward|{C}", f"[ADD]{A}|isHolding|{C}"]
        return sequence

    @staticmethod
    def Pick_In(A, C, O):
        grasp_sequence = Action.grasp(A, O)
        grasp_sequence.append(f"[DEL]{O}|isIn|{C}")
        return grasp_sequence

    @staticmethod
    def Pick_Over(A, O, S):
        grasp_sequence = Action.grasp(A, O)
        grasp_sequence.append(f"[DEL]{O}|isOnTopOf|{S}")
        return grasp_sequence

    @staticmethod
    def Place_In(A, O, S):
        release_sequence = Action.release(A, S, O)
        release_sequence.append(f"[ADD]{O}|isInContainer|{S}")
        return release_sequence

    @staticmethod
    def Place_Over(A, O, S):
        release_sequence = Action.release(A, S, O)
        release_sequence.append(f"[ADD]{O}|isOnTopOf|{S}")
        return release_sequence

    @staticmethod
    def Give(A, A2, C):
        grasp_sequence = Action.grasp(A, C)
        grasp_sequence.append(f"[DEL]{A2}|isHolding|{C}")
        return grasp_sequence

    @staticmethod
    def SplitEggs(Y, W, EY, EW, E, A):
        instructions = [
            f"[ADD]{EY}|isIn|{Y}",
            f"[ADD]{EW}|isIn|{W}",
            f"[ADD]{A}|isHolding|{E}"
        ]
        return instructions

    @staticmethod
    def Dispose(A, B, S):
        instructions = [
            f"[ADD]{A}|hasInHand|{B}",
            f"[ADD]{S}|hasSpreadOver|{B}",
            f"[DEL]{B}|hasIn|~"
        ]
        return instructions

    @staticmethod
    def PoorBatter(A, M, B):
        instructions = [
            f"[ADD]{A}|hasInHand|{M}",
            f"[ADD]{B}|hasIn|{M}",
            f"[DEL]{M}|hasIn|~"
        ]
        return instructions

    @staticmethod
    def PoorMold(A, P, M):
        instructions = [
            f"[ADD]{A}|hasInHand|{P}",
            f"[ADD]{M}|hasIn|{P}",
            f"[DEL]{M}|hasIn|~"
        ]
        return instructions

    @staticmethod
    def Cut(A, K, I):
        instructions = [
            f"[ADD]{A}|hasInHand|{K}",
            f"[ADD]{I}|isUnder|{K}",
            f"[ADD]{I}|isCutBy|{K}"
        ]
        return instructions

    @staticmethod
    def Twist(A, C, F):
        instructions = [
            f"[ADD]{A}|hasInHand|{F}",
            f"[ADD]{F}|isHoldIn|{C}",
            f"[ADD]{A}|CircularMove|{F}"
        ]
        return instructions