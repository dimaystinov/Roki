from roki_ng.body.imu_status import ImuDisplayStatus, read_status


def sample(q=(0,0,0,1), valid=True, sequence=1):
    return dict(valid=valid, quaternion_xyzw=q, sequence=sequence)


def test_unchanged_reads_do_not_prove_sensor_updates(tmp_path):
    p=tmp_path/'status.json'; w=ImuDisplayStatus(p)
    w.update(sample(),10)
    assert read_status(p,10)['state']=='checking'
    w.update(sample(sequence=99),14)
    assert read_status(p,14)['state']=='suspect'
    w.update(sample((.001,0,0,1),sequence=100),14.5)
    assert read_status(p,14.5)['state']=='updating'


def test_missing_invalid_or_dead_writer_never_shows_green(tmp_path):
    p=tmp_path/'status.json'
    assert read_status(p,10)['state']=='no_data'
    w=ImuDisplayStatus(p);w.update(sample(),10)
    w.update(sample((.001,0,0,1)),10.5)
    assert read_status(p,13)['state']=='no_data'
    w.update(sample(valid=False),13.5)
    assert read_status(p,13.5)['state']=='no_data'
    p.write_text('{bad')
    assert read_status(p,14)['state']=='no_data'


def test_stationary_orientation_with_sensor_progress_is_green_then_stall_is_red(tmp_path):
    p=tmp_path/'status.json';w=ImuDisplayStatus(p)
    w.update(sample() | {'sensor_sequence':1},10)
    w.update(sample() | {'sensor_sequence':2},14)
    assert read_status(p,14)['state']=='updating'
    w.update(sample(valid=False) | {'sensor_sequence':2, 'measurement_stalled':True},15)
    assert read_status(p,15)['state']=='stalled'
