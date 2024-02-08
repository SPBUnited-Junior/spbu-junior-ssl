import math
import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as field
import bridge.processors.entity as entity
import bridge.processors.waypoint as wp
import bridge.processors.tau as tau
from typing import List

class Robot(entity.Entity):


    def __init__(self, pos, angle, R, color, r_id, ctrl_id):
        super().__init__(pos, angle, R)

        self.rId = r_id
        self.ctrlId = ctrl_id
        self.isUsed = 0
        self.color = color
        self.lastUpdate = 0
        self.errOld = 0

        self.speedX = 0
        self.speedY = 0
        self.speedR = 0
        self.kickUp = 0
        self.kickForward = 0
        self.autoKick = 0
        self.kickerVoltage = 0
        self.dribblerEnable = 0
        self.speedDribbler = 0
        self.kickerChargeEnable = 1
        self.beep = 0   
        self.role = None

        #v! SIM
        if const.IS_SIMULATOR_USED:
            self.Kxx = -833/20
            self.Kyy = 833/20
            self.Kww = 1.25/20
            self.Kwy = -0.001
            self.Twy = 0.15
            self.RcompFdy = tau.FOD(self.Twy, const.Ts)
            self.RcompFfy = tau.FOLP(self.Twy, const.Ts)

        #v! REAL
        else:
            self.Kxx = -250/20
            self.Kyy = 250/20
            self.Kww = 6/20
            self.Kwy = 0
            self.Twy = 0.15
            self.RcompFdy = tau.FOD(self.Twy, const.Ts)
            self.RcompFfy = tau.FOLP(self.Twy, const.Ts)

        # self.max_acc = 1000
        # self.xxRL = tau.RateLimiter(const.Ts, self.max_acc)
        # self.yyRL = tau.RateLimiter(const.Ts, self.max_acc)
        self.xxTF = 0.2
        self.xxFlp = tau.FOLP(self.xxTF, const.Ts)
        self.yyTF = 0.2
        self.yyFlp = tau.FOLP(self.yyTF, const.Ts)
        # self.a0TF = 0.5
        # self.a0Flp = tau.FOLP(self.a0TF, const.Ts)

        #!v REAL
        # if self.rId != const.GK:
        gains_full = [6, 0.8, 0, const.MAX_SPEED]
        gains_soft = [8, 0.5, 0, const.SOFT_MAX_SPEED]
        # gains_soft = gains_full
        a_gains_full = [4, 0.1, 0.1, const.MAX_SPEED_R]
        a_gains_soft = [4, 0.07, 4, const.SOFT_MAX_SPEED_R]
        # a_gains_soft = a_gains_full
        # else:
        #     gains_full = [6, 0.8, 0, const.MAX_SPEED]
        #     gains_soft = [6, 1, 0.1, const.SOFT_MAX_SPEED]
        #     a_gains_full = [6, 0.1, 0, const.MAX_SPEED_R]
        #     a_gains_soft = [2, 0.07, 1, const.SOFT_MAX_SPEED_R]

        #!v SIM
        # gains_full = [2, 0.3, 0, const.MAX_SPEED]
        # gains_soft = [0.5, 0.1, 0, const.SOFT_MAX_SPEED]
        # a_gains_full = [2, 0.1, 0.1, const.MAX_SPEED_R]
        # a_gains_soft = [1, 0.07, 0, const.SOFT_MAX_SPEED_R]

        self.posReg = tau.PISD(const.Ts, [gains_full[0], gains_soft[0]], [gains_full[1], gains_soft[1]], [gains_full[2], gains_soft[2]], [gains_full[3], gains_soft[3]])
        self.angleReg = tau.PISD(const.Ts, [a_gains_full[0], a_gains_soft[0]], [a_gains_full[1], a_gains_soft[1]], [a_gains_full[2], a_gains_soft[2]], [a_gains_full[3], a_gains_soft[3]])

        self.is_kick_commited = False

    def used(self, a):
        self.isUsed = a

    def is_used(self):
        return self.isUsed
    
    def last_update(self):
        return self.lastUpdate

    def update(self, pos, angle, t):
        super().update(pos, angle)
        self.kickForward = 0
        self.kickUp = 0
        self.lastUpdate = t

    def kick_forward(self):
        self.kickForward = 1

    def kick_up(self):
        self.kickUp = 1

    def copy_control_fields(self, robot):
        self.speedX = robot.speedX
        self.speedY = robot.speedY
        self.speedR = robot.speedR
        self.kickUp = robot.kickUp
        self.kickForward = robot.kickForward
        self.autoKick = robot.autoKick
        self.kickerVoltage = robot.kickerVoltage
        self.dribblerEnable = robot.dribblerEnable
        self.speedDribbler = robot.speedDribbler
        self.kickerChargeEnable = robot.kickerChargeEnable
        self.beep = robot.beep

    def clear_fields(self):
        self.speedX = 0
        self.speedY = 0
        self.speedR = 0
        self.kickUp = 0
        self.kickForward = 0
        self.autoKick = 0
        self.kickerVoltage = 0
        self.dribblerEnable = 0
        self.speedDribbler = 0
        self.kickerChargeEnable = 0
        self.beep = 0
    
    def is_kick_aligned(self, target: wp.Waypoint):
        # print(round((self.getPos() - target.pos).mag(), 2), const.KICK_ALIGN_DIST*const.KICK_ALIGN_DIST_MULT, \
        #     round(abs(aux.wind_down_angle(self._angle - target.angle)), 2), const.KICK_ALIGN_ANGLE, \
        #     # round(abs(aux.vect_mult(aux.rotate(aux.i, target.angle), target.pos - self._pos)), 2), const.KICK_ALIGN_OFFSET)
        #     round(aux.dist(aux.closest_point_on_line(target.pos, target.pos - aux.rotate(aux.i, target.angle)*const.KICK_ALIGN_DIST, self._pos), self._pos)), const.KICK_ALIGN_OFFSET)
        # print(aux.i, target.angle, aux.rotate(aux.i, target.angle), target.pos, self._pos, target.pos - self._pos)

        if self.is_kick_commited:
            commit_scale = 1.2
            is_dist = (self.getPos() - target.pos).mag() < const.KICK_ALIGN_DIST*const.KICK_ALIGN_DIST_MULT * commit_scale
            is_angle = abs(aux.wind_down_angle(self._angle - target.angle)) < const.KICK_ALIGN_ANGLE * commit_scale
            is_offset = aux.dist(aux.closest_point_on_line(target.pos, target.pos - aux.rotate(aux.i, target.angle)*const.KICK_ALIGN_DIST, self._pos), self._pos) < const.KICK_ALIGN_OFFSET * commit_scale
            is_aligned = is_dist and is_angle and is_offset
        else:
            commit_scale = 1
            is_dist = (self.getPos() - target.pos).mag() < const.KICK_ALIGN_DIST*const.KICK_ALIGN_DIST_MULT * commit_scale
            is_angle = abs(aux.wind_down_angle(self._angle - target.angle)) < const.KICK_ALIGN_ANGLE * commit_scale
            is_offset = aux.dist(aux.closest_point_on_line(target.pos, target.pos - aux.rotate(aux.i, target.angle)*const.KICK_ALIGN_DIST, self._pos), self._pos) < const.KICK_ALIGN_OFFSET * commit_scale
            is_aligned = is_dist and is_angle and is_offset
        if is_aligned:
            self.is_kick_commited = True
        else:
            self.is_kick_commited = False
        #print(is_dist, is_angle, is_offset)
        return is_aligned
        
    def is_ball_in(self, field: field.Field):
        return (self.getPos() - field.ball.getPos()).mag() < const.BALL_GRABBED_DIST and \
            abs(aux.wind_down_angle((field.ball.getPos() - self.getPos()).arg() - self._angle)) < const.BALL_GRABBED_ANGLE

    def go_route(self, route, field: field.Field):
        cur_speed = self._vel.mag()

        # print(route)

        dist = route.getLenght()

        target_point = route.getNextWP()
        end_point = route.getDestWP()
        
        # print(self.getPos(), target_point, end_point)
        vel0 = (self.getPos() - target_point.pos).unity()

        dangle = (target_point.pos - self.getPos()).arg()
        rangle = aux.wind_down_angle(self._angle - dangle)
        twpangle = aux.wind_down_angle(target_point.angle - dangle)

        angle60_abs = math.pi/6 if abs(rangle) < math.pi/2 else 2*math.pi/6
        # angle60_abs = 0
        angle60_sign = aux.sign(twpangle + rangle)

        angle60 = dangle + angle60_abs * angle60_sign

        tpangle = target_point.angle if target_point.type != wp.WType.R_PASSTHROUGH else end_point.angle
        lerp_angles = [target_point.angle, angle60]

        angle0 = aux.LERP(lerp_angles[0], lerp_angles[1], aux.minmax((dist-100)/1000, 0, 1))

        self.posReg.select_mode(tau.Mode.NORMAL)

        if (end_point.type == wp.WType.S_BALL_KICK or \
            end_point.type == wp.WType.S_BALL_GRAB or \
            end_point.type == wp.WType.S_BALL_GO) and \
                dist < 1500:

            #print("IS KICK ALIGNED: ", self.is_kick_aligned(end_point), ",\tIS BALL GRABBED: ", self.is_ball_in(field))
            
            self.posReg.select_mode(tau.Mode.SOFT)

            if end_point.type == wp.WType.S_BALL_GO:
                angle0 = end_point.angle

            self.dribblerEnable = True
            self.speedDribbler = 15
            self.kickerVoltage = 7
        else:
            self.dribblerEnable = False

        if (end_point.type == wp.WType.S_BALL_KICK or end_point.type == wp.WType.S_BALL_GRAB) and \
            (self.is_kick_aligned(end_point) or self.is_ball_in(field)):
            # vel0 = (self.getPos() - end_point.pos).unity()
            vel0 = -aux.rotate(aux.i, self._angle)
            # angle0 = end_point.angle
            angle0 = self._angle

            if end_point.type == wp.WType.S_BALL_KICK:
                self.autoKick = 1
                # self.autoKick = 2 if self.rId == const.GK else 1
                # if self._pos.x * field.side > 500:
                #     self.autoKick = 2
                # else:
                #     self.autoKick = 1
            else:
                self.autoKick = 0

            transl_vel = vel0 * 300

        else:
            self.autoKick = 0

            err = dist
            u = self.posReg.process(err, -cur_speed)
            transl_vel = vel0 * u

        angle0 = end_point.angle
        aerr = aux.wind_down_angle(angle0 - self.getAngle())

        u_a = self.angleReg.process(aerr, -self._anglevel)
        ang_vel = u_a

        # if self.rId == const.GK and self.color == 'b':
        #     print("is aligned: ", self.is_kick_aligned(end_point), ", angle60: ", '%.2f'%angle60, ", end angle: ", '%.2f'%end_point.angle, \
        #           ", angle0: ", '%.2f'%angle0, ", aerr: ", '%.2f'%aerr, ", self angle: ", '%.2f'%self.getAngle())
        self.update_vel_xyw(transl_vel, ang_vel)

    def update_vel_xyw(self, vel: aux.Point, wvel: float):
        """
        Выполнить тик регуляторов скорости робота

        vel - требуемый вектор скорости [мм/с] \\
        wvel - требуемая угловая скорость [рад/с]
        """
        self.speedX = self.xxFlp.process(1/self.Kxx * aux.rotate(vel, -self._angle).x)
        self.speedY = self.yyFlp.process(1/self.Kyy * aux.rotate(vel, -self._angle).y)

        # RcompY = self.Kwy * self.RcompFfy.process(self.RcompFdy.process(self.speedY))
        # RcompY = self.Kwy * self.RcompFdy.process(abs(float(self.speedY)**2))
        RcompY = 0
        self.speedR = 1/self.Kww * (wvel - RcompY)
        if abs(self.speedR) > const.MAX_SPEED_R:
            self.speedR = const.MAX_SPEED_R * abs(self.speedR) / self.speedR

        vecSpeed = math.sqrt(self.speedX ** 2 + self.speedY ** 2)
        rSpeed = abs(self.speedR)

        vecSpeed *= ((const.MAX_SPEED_R - rSpeed) / const.MAX_SPEED_R) ** 8
        ang = math.atan2(self.speedY, self.speedX)
        self.speedX = vecSpeed * math.cos(ang)
        self.speedY = vecSpeed * math.sin(ang)
        
    def clamp_motors(self):
        """
        Ограничить управляющее воздействие"""

    def __str__(self) -> str:
        return str(str(self.color) + " " + str(self.rId) + " " + str(self.getPos()) + " " + str(self.speedX) + " " + str(self.speedY)) + " " + str(self.speedR)

