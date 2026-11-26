# Паузы походки: диагностика 22 ноября 2026

В runtime опрос IMU выполнялся до получения следующей инструкции генератора.
После отправки servo текущая instruction=None. При наступившем next_at прежняя
проверка только instruction[0]==servo не запрещала блокирующее чтение датчиков.
Добавлен motion_due для ожидающего генератора или servo; explicit imu wait
не блокируется. Исправление применено локально и на роботе, без переноса
остальных локальных экспериментальных изменений runtime.

Тест регрессии воспроизвёл проблему для BodyImu и FallbackImu; после исправления
57 passed, 30 skipped (нативная кинематика недоступна в данном Python Mac).

На неподвижном роботе, при остановленном roki-head, выполнено 100 пар чтений
реального контроллера: IMU 24 байта и позиции 30 серв 60 байт.
- IMU: среднее 0.927 мс, максимум 1.642 мс.
- Позиции: среднее 1.308 мс, максимум 1.959 мс.
- Вместе: среднее 2.235 мс, максимум 3.007 мс.

Сервис восстановлен. Это подтверждает лишнюю задержку перед отправкой,
но само по себе не объясняет большие видимые остановки. Интервалы реальных
servo-пакетов под нагрузкой ещё не измерены; остаются явные drain между циклами,
ожидания новых IMU-отсчётов, период worker и время отправки. Нельзя объявлять
рывки устранёнными по одному тесту приоритета. Движение в этом измерении не запускалось.

Исходные измерения: output/gait-timing-20261122/reads.json.

## Actual frame timing and field retry

Measured real `_send_targets` timestamps using a bounded in-memory trace,
flushed only at job completion. Same manual command: speed 0.35, heading hold,
2.5 seconds of held input, target body tilt 0.065. The duration of held input
includes preparation, so number of completed cycles is not identical.

Before: `output/gait-timing-20261122/gait-timing--30.json`, 94 sends.
Normal frame interval mean 29.23 ms (23–36 ms); cycle gaps about 102–104 ms.
Transport send maximum 6.40 ms. Trace shows idle worker polling, drain retries,
and extra loop iterations for drain/IMU instructions contributing to gaps.

Changes shared by all Body motion plans:
- Worker select timeout follows the next motion deadline, bounded 1–10 ms.
- Servo deadline measured from send start, not send completion; no accumulated
  deadline catch-up or bursts after a late frame.
- Unfinished drain is checked after 5 ms instead of 20 ms; controller drain
  semantics and timeout retained.

After: `output/gait-timing-20261122/after.json`, 114 sends. Normal intervals
20.52–27.95 ms, mean 23.12 ms. Interior cycle gaps 71–77 ms; final gaps 53–54 ms.
The intentional initial 200 ms and final 800 ms slot interpolation are not
classified as gait stalls. Two short physical passes completed without falls
or monitor interruption. Pitch ranges before -5.36..4.02 degrees, after
-7.37..2.73 degrees. This is evidence of improved scheduling, NOT evidence
that balance is fixed. Residual backwards sway and cycle boundary settling
still require investigation. No long run or autonomous football validation.

Video/IMU: `output/field-tilt-slew-20261122-152719.*` and
`output/field-tilt-slew-20261122-152843.*`.
Temporary runtime trace removed after measurement. No firmware changes.
Regression suite: 56 passed, 7 skipped (Mac native-backend limitations).
