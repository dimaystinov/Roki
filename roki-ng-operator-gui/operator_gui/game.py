"""Explicit autonomous game controls; connection alone never starts a game."""
from datetime import datetime, timezone
import json
import math
import time

from PySide6.QtCore import QObject, Property, QTimer, Signal, Slot


def phase_label(phase):
    groups = (
        ('Проверка результата удара', ('verify', 'kick_moved', 'kick_missed', 'kick_unconfirmed', 'pass_retry')),
        ('Удар прыжком вперёд', ('strike_jump',)),
        ('Удар ногой', ('strike_foot', 'visual_opening_kick', 'angled_kick')),
        ('Подготовка к удару', ('kick_prepare', 'toe_check', 'pass_heading', 'fine_heading')),
        ('Вставание', ('get_up',)),
        ('Игра остановлена', ('stopped',)),
        ('Пауза', ('paused',)),
        ('Ожидание старта', ('start_later',)),
        ('Поиск ворот', ('goal_search', 'goal_scan')),
        ('Мяч перекрыт · подход по прогнозу', ('ball_prediction', 'predicted_approach')),
        ('Обход соперника', ('jump_detour',)),
        ('Прицеливание по воротам', ('keeper_view', 'aim_check', 'aim_adjust')),
        ('Ближний подход', ('visual_opening_jump', 'toe_hop', 'test_fine', 'side_jump')),
        ('Дальний подход', ('walk_to_ball', 'test_approach', 'search_walk')),
        ('Поиск по курсу ворот · нет изображения', ('search_no_image',)),
        ('Поиск мяча', ('search', 'ball_recheck', 'ball_unseen')),
        ('Попытка касания по стартовой позиции', ('expected_touch',)),
        ('Стартовый розыгрыш', ('kickoff',)),
    )
    return next((name for name, tokens in groups if any(token in phase for token in tokens)), phase)


class Game(QObject):
    changed = Signal()

    def __init__(self, session, control, data_sources, parent=None):
        super().__init__(parent)
        self.session, self.control = session, control
        self.data_sources = data_sources
        data_sources.sampleReceived.connect(self._sample)
        data_sources.changed.connect(self.changed.emit)
        self.strikes = {}
        self.state = {}
        self.calibration_enabled = False
        self.visual_localisation = None
        self.error = ""
        self.pending = False
        self.received = 0.
        self.archive = None
        self.archive_status = "Ожидание данных"
        self.clock = QTimer(self)
        self.clock.setInterval(250)
        self.clock.timeout.connect(self.changed.emit)
        self.clock.start()
        session.response.connect(self._response)
        session.failed.connect(self._failed)
        session.changed.connect(self._connection)
        control.changed.connect(self.changed.emit)
        control.barrierIssued.connect(self.barrier)

    @Property('QVariantMap', notify=changed)
    def view(self):
        fresh = self.session.connected and self.received > 0 and time.monotonic()-self.received < 2
        ball = self.state.get('ball')
        running = self.state.get('running') is True
        pickup = self.state.get('pickup') is True
        paused = self.state.get('paused') is True
        reentry = fresh and running and pickup and self.state.get('pickup_ready') is True
        available = self.control.owns and not self.control.pending and not self.control.uncertain
        active = available and fresh and running and self.control.mode == 'GAME'
        blocked = self.control.blocked_reason
        if self.session.connected and self.control.mode == 'GAME':
            if fresh and self.state.get('running'):
                blocked = ('Можно выбрать роль и место повторного ввода.' if reentry else
                           'Игра уже запущена. Для нового ввода используйте Pick up или остановите игру.')
                if not self.control.owns:
                    blocked += ' Для остановки получите управление.'
            else:
                blocked = 'Робот в режиме GAME. Нажмите «Запросить статус», чтобы узнать состояние игры.'
        if self.control.mode == 'IDLE' and self.control.view['canEnterManual']:
            blocked = 'Для запуска игры включите ручной режим, выберите роль и нажмите «Запустить игру».'
        if self.calibration_enabled:
            blocked = 'Для запуска игры выберите «Футбол» на вкладке «Тесты».'
        setup = self.state.get('kick_setup') if fresh and running and not paused and not pickup else None
        placement = ''
        if setup:
            legs = {'left': 'левая', 'right': 'правая'}
            front, side = setup['front_error_mm'], setup['side_error_mm']
            reach = 'в зоне удара' if setup['in_reach'] else 'вне зоны удара'
            placement = (f"Мяч на кадре {setup.get('frame') or '—'} · ударная {legs[setup['leg']]}, "
                         f"опорная {legs[setup['support']]} · {reach}. "
                         f"Центр мяча от корпуса: X {setup['x_mm']} мм, Y {setup['y_mm']} мм. "
                         f"От точки удара: {abs(front)} мм {'дальше' if front >= 0 else 'ближе'}, "
                         f"{abs(side)} мм {'левее' if side >= 0 else 'правее'}.")
        phase = self.state.get('phase') or '—'
        return dict(phaseLabel=phase_label(phase) if fresh else 'Нет свежих данных',
                    archiveStatus=self.archive_status, strikes=dict(self.strikes),
                    running=running, fresh=fresh, kickPlacement=placement,
                    visualLocalisationKnown=self.visual_localisation is not None,
                    visualLocalisation=self.visual_localisation is True,
                    state=self.state.get('state', 'Статус не запрошен'),
                    role=self.state.get('role') or '—', phase=self.state.get('phase') or '—',
                    recovery=self.state.get('recovery') or 'none',
                    recoveryAttempt=self.state.get('recovery_attempt', 0),
                    paused=paused, pickup=pickup, pickupReady=fresh and self.state.get('pickup_ready') is True,
                    confirmationRequired=fresh and self.state.get('pickup_confirmation_required') is True,
                    position=json.dumps(self.state.get('position'), ensure_ascii=False) if fresh else '—',
                    correction=json.dumps(self.state.get('last_correction'), ensure_ascii=False) if fresh else '—',
                    reason=self.state.get('reason') or '', error=self.error,
                    decision=(self.state.get('selected_motion') or self.state.get('decision', 'hold')) if fresh and running and not paused and not pickup else '—',
                    ball=json.dumps(ball, ensure_ascii=False) if fresh and ball else 'Нет актуального мяча',
                    travel=self.state.get('travel_m', 0), jobId=self.state.get('job_id') or '—',
                    pending=self.pending, canRefresh=self.session.connected and not self.pending,
                    watching='game.state' in self.data_sources.wanted,
                    subscriptionPending=any(t == 'game.state' for _, t in self.data_sources.pending),
                    canStart=not self.calibration_enabled and ((not self.control.blocked_reason and not running) or (active and reentry)),
                    canPause=active and not paused and not pickup,
                    canResume=active and paused and not pickup,
                    canPickup=self.control.owns and self.control.mode == 'GAME' and not pickup
                        and self.control.pending not in ('game.pickup', 'game.stop', 'motion.stop_hard',
                                                         'control.release', 'control.acquire', 'mode.set'),
                    canConfirm=active and pickup and self.state.get('pickup_confirmation_required') is True,
                    canStop=self.control.owns and self.control.pending != 'game.stop',
                    blockedReason=blocked)

    @Slot()
    def loadSettings(self):
        if self.session.connected:
            for stage in ('opening', 'play'):
                self.session.request('params.get', {'key': f'game.simple_football.{stage}_strike'}, 'game:strike:'+stage)
            self.session.request('params.get', {'key': 'game.use_visual_localisation'}, 'game:visual-setting')

    @Slot(str, str)
    def setStrike(self, stage, method):
        if stage in self.strikes and method in ('strategy', 'foot', 'jump') and self.view['canStart']:
            self.control.command('params.set', {'key': f'game.simple_football.{stage}_strike', 'value': method},
                                 manual=False, job=False, context='game:strike:'+stage)

    def record(self, kind, data):
        if self.archive_status.startswith('Ошибка'):
            return
        try:
            if self.archive is None:
                directory = self.parent().config_dir / 'match-logs'
                directory.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
                self.archive = (directory / (stamp+'.jsonl')).open('x', encoding='utf-8', buffering=1)
                self.archive_status = str(self.archive.name)
            row = dict(time=datetime.now(timezone.utc).isoformat(), kind=kind, data=data)
            self.archive.write(json.dumps(row, ensure_ascii=False)+'\n')
        except OSError as exc:
            self.archive_status = 'Ошибка записи: '+str(exc)

    @Slot(bool)
    def setVisualLocalisation(self, enabled):
        if self.visual_localisation is None or not self.view['canStart']:
            return
        self.control.command('params.set', {'key': 'game.use_visual_localisation', 'value': enabled,
                             'expected_value': self.visual_localisation},
                             manual=False, job=False, context='game:visual-setting')

    @Slot(str, str, float)
    @Slot(str, str, float, str)
    def start(self, strategy='FIRA_penalty_Goalkeeper', entry='center', delay_seconds=0., kick_direction='straight'):
        body = dict(strategy=strategy)
        if strategy == 'danger_football':
            body['kick_direction'] = kick_direction
        if strategy in ('forward', 'simple_football', 'danger_football'):
            body['entry'] = entry
        if (isinstance(delay_seconds, bool) or not math.isfinite(delay_seconds)
                or not float(delay_seconds).is_integer() or not 0 <= delay_seconds <= 30):
            self.error = 'Задержка должна быть целым числом от 0 до 30 секунд.'
        elif strategy not in ('forward', 'FIRA_penalty_Goalkeeper', 'ball_kick_test', 'simple_football', 'danger_football', 'simple_goalkeeper') or entry not in ('center', 'left', 'right'):
            self.error = 'Неизвестная роль или место ввода.'
        elif kick_direction not in ('straight', 'left', 'right', 'kickoff_left'):
            self.error = 'Неизвестное направление удара.'
        elif not self.view['canStart']:
            self.error = self.view['blockedReason'] or 'Запуск сейчас недоступен.'
        elif self.control.command('game.start', body | dict(delay_seconds=int(delay_seconds)),
                                  manual=self.control.mode != 'GAME', context='game:start'):
            self.error = ''
        else:
            self.error = self.control.error
        self.changed.emit()

    @Slot()
    def pause(self):
        if self.view['canPause']:
            self._barrier('game.pause')

    @Slot()
    def resume(self):
        if self.view['canResume']:
            self.control.command('game.resume', {}, manual=False, job=False, context='game:resume')

    @Slot()
    def pickup(self):
        if self.view['canPickup']:
            self._barrier('game.pickup')

    @Slot()
    def confirmUpright(self):
        if self.view['canConfirm']:
            self.control.command('game.pickup', {'confirm': True}, manual=False, job=False,
                                 context='game:confirm')

    def _barrier(self, op):
        self.control.stopInput()
        self.control.barrierIssued.emit()
        self.control.pending = op
        self.control.poll_pending = False
        self.error = ''
        self.session.interrupt(op, {}, 'game:' + op.split('.')[1])
        self.control.changed.emit()

    @Slot()
    def stop(self):
        if not self.control.owns or self.control.pending == 'game.stop':
            return
        self._barrier('game.stop')

    @Slot()
    def refresh(self):
        if self.session.connected and not self.pending:
            self.pending = True
            self.session.request('game.status', {}, 'game:status')
            self.changed.emit()

    @Slot(bool)
    def watch(self, enabled):
        if enabled:
            self.data_sources.subscribe_topic('game.state', 2)
        else:
            self.data_sources.unsubscribe_topic('game.state')

    def _sample(self, topic, sample):
        if topic != 'game.state':
            return
        if self.parent() is not None:
            self.record('game.state', sample)
        self.state = dict(sample['data'])
        age = sample.get('age_ms')
        valid_age = type(age) in (int, float) and math.isfinite(age) and age >= 0
        self.received = time.monotonic() - age / 1000 if sample.get('valid') is True and valid_age else 0.
        self.changed.emit()

    def barrier(self):
        self.pending = False

    def _connection(self):
        if not self.session.connected:
            self.state = {}
            self.calibration_enabled = False
            self.strikes.clear()
            self.closeArchive()
            self.visual_localisation = None
            self.received = 0.
            self.pending = False
        self.changed.emit()

    def _response(self, op, result, context):
        if context.startswith('game:strike:'):
            self.strikes[context.split(':')[-1]] = result['value']
            self.changed.emit()
            return
        if op.startswith('sim.calibration.'):
            enabled = result.get('enabled') is True
            if enabled != self.calibration_enabled:
                self.calibration_enabled = enabled
                self.refresh()
                self.changed.emit()
            return
        if context == 'game:visual-setting' and op in ('params.get', 'params.set'):
            self.visual_localisation = result['value']
            self.error = ''
            self.changed.emit()
            return
        if op not in ('game.start', 'game.stop', 'game.status', 'game.pause', 'game.resume', 'game.pickup'):
            return
        if op == 'game.status':
            self.pending = False
        self.state = dict(result)
        self.received = time.monotonic()
        if op != 'game.status':
            self.error = ''
        if op in ('game.start', 'game.stop'):
            self.session.request('system.status')
        self.changed.emit()

    def _failed(self, op, message, context):
        if context == 'game:visual-setting' or context.startswith('game:strike:'):
            self.error = message
            self.changed.emit()
            return
        if op.startswith('data.') and context == 'game.state':
            self.error = message
            self.changed.emit()
            return
        if op.startswith('game.'):
            self.pending = False
            self.error = message
            if op == 'game.start' and 'outcome unknown' in message:
                self.refresh()
            self.changed.emit()

    def closeArchive(self):
        if self.archive is not None:
            self.archive.close()
            self.archive = None
            self.archive_status = "Ожидание данных"

    def shutdown(self):
        self.closeArchive()
        self.clock.stop()
