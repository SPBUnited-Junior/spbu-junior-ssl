import bridge.router.waypoint as wp
from bridge.auxiliary import fld

gk_idx = 0
idx1 = 1
idx2 = 2


def run(field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
    # attacker(field, waypoints, attacker_idx)
    # goalkeeper(field, waypoints)
    return None


def attacker(field: fld.Field, waypoints: list[wp.Waypoint], idx: int) -> None:
    # Здесь будет твой код для главного атакующего (робота, который едет бить мяч)
    # Эту роль можно передавать между роботами idx1 и idx2
    return None


def goalkeeper(field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
    # Здесь будет твой код для вратаря с индексом gk_idx
    # Эту роль нельзя передавать другим роботам, индекс вратаря сообщается судье и противникам до начала матча
    return None
