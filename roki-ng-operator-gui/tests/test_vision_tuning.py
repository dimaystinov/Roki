from types import SimpleNamespace
import unittest

from PySide6.QtCore import QCoreApplication
from operator_gui.vision_tuning import VisionTuning
from tests.test_field_editor import Session,Control


class TuningTests(unittest.TestCase):
    def test_colour_diagnostic_requires_loaded_profile_but_ball_does_not(self):
        self.control.mode='MANUAL'
        self.model.profile='';self.model.profiles=[]
        self.model.action('detection.start')
        self.assertFalse(self.control.commands)
        self.assertIn('загрузите настройки',self.model.notice)
        self.model.selectDetector('ball')
        self.model.action('detection.start')
        self.assertEqual(self.control.commands[-1][0],('detection.start',{'mode':'ball'}))

    def test_ball_diagnostic_uses_explicit_mode_and_refuses_active_game(self):
        self.control.mode='MANUAL'
        self.model.selectDetector('ball')
        self.model.action('detection.start')
        args,kwargs=self.control.commands[-1]
        assert args==('detection.start',{'mode':'ball'})
        assert kwargs['manual'] and not kwargs['job']
        self.control.commands.clear();self.control.mode='GAME'
        self.model.action('detection.start')
        assert not self.control.commands and 'Остановите игру' in self.model.notice

    def test_football_diagnostics_explicit_mode_without_motion(self):
        self.control.mode = 'MANUAL'
        self.model.selectDetector('football')
        self.model.action('detection.start')
        args, kwargs = self.control.commands[-1]
        assert args == ('detection.start', {'mode': 'football'})
        assert not kwargs['job']
        self.control.commands.clear()
        self.control.mode = 'GAME'
        self.model.action('detection.start')
        assert not self.control.commands

    def test_start_ack_enables_status_watch_without_camera_or_motion_commands(self):
        self.model.response('detection.start',{'mode':'ball','running':True},'tuning:action')
        assert self.model.view['watching'] and self.model.detector['running']
        assert not self.control.commands

    def test_detector_can_start_in_game(self):
        self.control.mode = 'GAME'
        self.model.action('detection.start')
        args, kwargs = self.control.commands[-1]
        self.assertEqual(args[0], 'detection.start')
        self.assertFalse(kwargs['manual'])
    @classmethod
    def setUpClass(cls):cls.app=QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.session=Session();self.control=Control()
        self.model=VisionTuning(self.session,self.control,SimpleNamespace())
        self.model.profile='green_field';self.model.profiles=['green_field']
        for axis,lo,hi in [('l',0,100),('a',-128,127),('b',-128,127)]:
            for suffix,value in [('min',lo),('max',hi)]:
                key=f'vision.green_field.{axis}_{suffix}'
                self.model.metas[key]={'type':'int','min':lo,'max':hi,'default':value}
                self.model.values[key]=value

    def test_text_transport_error_keeps_draft(self):
        self.model.edit('vision.green_field.l_min',20)
        self.model.failed('params.set','conflict: reload','tuning:save')
        assert self.model.drafts and 'conflict' in self.model.notice

    def test_status_poll_does_not_rebuild_slider_catalog(self):
        emitted=[];self.model.catalogChanged.connect(lambda:emitted.append(True))
        self.model.status()
        self.model.response('detection.status',{},'tuning:status')
        assert not self.model.busy and not emitted

    def test_catalog_page_reads_values_in_one_request(self):
        keys=list(self.model.values)
        self.model.response('params.keys',{'items':keys},'tuning:keys:vision.')
        scheduled=self.model.queue
        batches=[item for item in scheduled if item[0]=='params.get']
        assert batches==[('params.get',{'keys':keys},'tuning:load')]
        self.model.queue=[]
        self.model.response('params.get',{'values':{keys[0]:15}},'tuning:load')
        assert self.model.values[keys[0]]==15

    def tearDown(self):
        self.model.shutdown()

    def test_colour_selection_switches_running_lab_without_restart_or_saving_drafts(self):
        self.model.profiles.append('orange_ball')
        self.model.edit('vision.green_field.l_min',20)
        self.model.detector={'mode':'colour','running':True}
        self.model.select('orange_ball')
        args,kwargs=self.control.commands[-1]
        assert args==('detection.select',{'profile':'orange_ball'})
        assert not kwargs['manual'] and not kwargs['job']
        assert self.model.drafts=={'vision.green_field.l_min':20}
        self.control.commands.clear()
        self.model.detector={'mode':'football','running':True}
        self.model.select('green_field')
        assert not self.control.commands
