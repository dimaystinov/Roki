# Head IMU as body orientation: diagnostic implementation

`body/head_body_orientation.py` implements xyzw rotation composition:

    body_world = world_alignment * head_world * inverse(neck * mounting)
    neck = rotation(body_up_Y, pan) * rotation(body_pitch_X, tilt)

Mounting, axis signs, servo zero offsets and world yaw alignment MUST be verified
against paired measurements before enabling fallback for leg control. The
mathematical tests validate rotation order, noncommuting rotations, normalization
and rejection of invalid inputs, not the physical model of this robot.

Head `ImuPublisher` now retains the latest quaternion with its own sensor timestamp
and host receive time. Repeated sensor timestamps do not refresh that cache.
`body.imu.head_reference` exposes it together with commanded head targets for
read-only comparison. Existing capture/data-plane behavior is unchanged.

On the live robot the head sensor timestamp advanced from 14.83 to 17.06 seconds
while the body quaternion stayed bit-identical. Thus the head IMU is available;
this alone does not validate neck compensation. Runtime starts commanded head
values at zero after restart, so these targets are not a trustworthy physical
pose until explicitly commanded and settled or read back from servos. Native
Zubr head position units and tilt mirroring must be honored.

Fallback is NOT enabled. No gait was launched. Waiting for a healthy body IMU
session to compare multiple fixed head poses, measure prediction error on held-out
poses and verify time skew. Both sensors can have independent yaw frames; do not
blindly blend their quaternion components. Loss of both sources must still stop
motion, and recovery must not silently resume a cancelled job.

## Live validation after power cycle

Artifacts: `output/head-body-check-20261121-201421/samples.json` and `fit.json`.
Ten static head poses, including held-out pan/tilt combinations and repeated
neutral pose. Sensor receive-time mismatch required <=100 ms. Joint targets were
commanded and allowed to settle; this is not validation of moving-neck feedback.

Model fitted with world alignment restricted to yaw, fixed mounting rotation,
and tested signs: pan +1, tilt -1, servo radians = ticks / 1698.
Static train RMS 1.0105 degrees; held-out RMS 0.8087, max 1.0519 degrees.
Do not reuse the fitted world alignment across sensor resets: independent yaw
origins must be aligned again. Estimated mounting is a session candidate.

`output/head-body-shadow-walk-20261121-201506.jsonl/.mkv`: manual forward command
at speed .35 for 1.5 s, existing max step 24 mm (requested step 8.4 mm), followed
by settled stance. Final job none, pose base_stand, no fall indicated by IMU.
Pitch range -11.72..8.05 degrees; roll -9.87..8.54 degrees.

Head-to-body prediction during movement: 34 matched samples, RMS 0.8366 degrees,
max 1.7730 degrees. Rest: 52 samples, RMS 0.4149, max 0.6998 degrees.
The body IMU controlled this trial; head estimate was shadow-only. This is NOT a
forced-fallback gait test or proof of safe mid-step switching. Automatic source
selection and controlled source-failure testing remain outstanding.

## Автоматический резерв: проверка 21 ноября, 20:26

На 192.168.0.244 включён `FallbackImu`. Конфигурация конкретного экземпляра:
`/var/lib/roki-ng/head-body-imu.json` (`enabled`, `mounting_xyzw`,
`pan_sign=1`, `tilt_sign=-1`). Мировой yaw согласуется заново при запуске,
из сохранённого опыта не берётся. Основной IMU остаётся доступен в `primary`;
`active_source` показывает источник эффективной ориентации.

Условия резерва: живой поток IMU головы, согласование по 25 парам в покое,
известное установившееся положение шеи, проверенный диапазон pan ±300 ticks,
tilt от −1000 до 0 ticks. После перезапуска нужно запустить штатную камеру
и явно установить голову; до этого резерв не готов. Параметр
`head_fallback_ready` показывает текущую доступность, а не только наличие
калибровки. Автоматического запуска камеры этот патч не добавляет.

Счётчик IMU тела, если он доступен, определяет свежесть измерений. На текущем
Zubr счётчика нет: используются неизменность quaternion более секунды и
расхождение с компенсированной головой более 4°. Это эвристика, она не
доказывает исправность неподвижного датчика. После такого отказа требуется
изменение данных тела, чтобы считать его восстановившимся. Возврат с головы
на тело — после согласия менее 3° в течение секунды и завершения задания.
При отсутствии обоих достоверных источников эффективный IMU недействителен.

В режиме HEAD голова сохраняется неподвижной. Явная команда изменения головы
отклоняется, внутреннее завершение стойки пропускает её перестановку, но
заканчивает движение тела. Это ограниченный резерв, а не подтверждённая
работа при произвольном вращении шеи или выполнении вставания.

Экран IP показывает `IMU BODY`, `IMU HEAD` или `IMU NONE` с состоянием.
Проверен файл отображения `/run/roki-body-imu.json`: при резерве HEAD/UPDATING
и `counter_present=true`, что отрисовывается как `IMU HEAD: OK`.

Живой тест: короткий ручной проход, speed=0.35, tilt=0.065, подача движения
1.5 секунды. Временная диагностическая вставка исключала тело из выбора
на интервале 2–5 секунд, сохраняя настоящие показания тела для сравнения.
Вставка и её файл удалены после теста; в установленном runtime их нет.

Результат второго прохода:
- BODY → HEAD на 2.004 s; HEAD → BODY на 5.992 s.
- Невалидных выборок: 0; падения не зарегистрированы.
- Наклон pitch: −5.58…+7.32°.
- Расхождение головы и тела во время HEAD: RMS 0.745°, максимум 1.787°.
- Завершение: `base_stand`, `active_job=null`, `error=null`.
- Первая проба выявила конфликт блокировки головы с завершением стойки;
  исправлен и покрыт регрессионным тестом.

Артефакты в рабочей папке `output/head-fallback-short-walk-20261121-202621`:
`.jsonl`, `.summary.json`, `.mkv`, `.jpg`. Проверки IMU, polling, соединения,
геометрии и дисплея: 49 passed. Это проверка короткого прохода с имитацией
отказа; устойчивость Long run и работа при настоящем зависании датчика
после падения данным опытом не подтверждены.
