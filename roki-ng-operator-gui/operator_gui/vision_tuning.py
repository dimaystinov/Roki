"""Edit robot LAB thresholds; diagnostics are computed only on the robot."""
from PySide6.QtCore import QObject,Property,Signal,Slot,QTimer

from .control import scalar


class VisionTuning(QObject):
    changed=Signal()
    catalogChanged=Signal()
    def __init__(self,session,control,streams,parent=None):
        super().__init__(parent)
        self.session,self.control,self.streams=session,control,streams
        self.metas={};self.values={};self.drafts={};self.profiles=[];self.profile=''
        self.queue=[];self.busy=False;self.notice='Загрузите настройки с робота.'
        self.catalog_dirty=False
        self.detector={}
        self.detector_mode='colour'
        self.timer=QTimer(self);self.timer.setInterval(1000);self.timer.timeout.connect(self.status)
        session.response.connect(self.response);session.failed.connect(self.failed);session.changed.connect(self.connection)

    def keys(self,scope):
        prefix='vision.'+self.profile+'.'
        return [k for k in self.metas if k.startswith(prefix)]

    def rows(self,scope):
        return [dict(key=k,meta=self.metas[k],value=self.drafts.get(k,self.values.get(k))) for k in self.keys(scope)]

    @Property('QStringList',notify=catalogChanged)
    def labKeys(self):return self.keys('lab')

    @Property('QVariantMap',notify=changed)
    def view(self):
        return dict(profiles=self.profiles,profile=self.profile,labRows=self.rows('lab'),
            busy=self.busy,loading=self.catalog_dirty,notice=self.notice,detector=self.detector,detectorMode=self.detector_mode,
            dirty=bool(self.drafts),watching=self.timer.isActive(),metas=self.metas,values=self.values|self.drafts)

    @Slot()
    def connection(self):
        if not self.session.connected:
            self.timer.stop();self.queue=[];self.busy=False;self.catalog_dirty=False;self.metas={};self.values={};self.drafts={}
            self.profiles=[];self.profile='';self.detector={}
            self.notice='Нет соединения. Перечитайте настройки после подключения.';self.catalogChanged.emit();self.changed.emit()

    def next(self):
        if self.queue:
            self.busy=True;op,args,context=self.queue.pop(0);self.session.request(op,args,context)
        else:
            self.busy=False
            if self.catalog_dirty:self.catalog_dirty=False;self.catalogChanged.emit()
        self.changed.emit()

    @Slot()
    def refresh(self):
        if self.busy or not self.session.connected:return
        if self.drafts:self.notice='Сохраните или отмените черновик перед загрузкой.';self.changed.emit();return
        self.metas={};self.values={}
        self.catalog_dirty=True
        self.queue=[('detection.list',{},'tuning:profiles')]
        for prefix in ('vision.',):
            self.queue.append(('params.keys',{'prefix':prefix,'limit':8},'tuning:keys:'+prefix))
        self.next()

    @Slot()
    def status(self):
        if self.busy or not self.session.connected:return
        self.queue=[('detection.status',{},'tuning:status')];self.next()

    @Slot(bool)
    def watch(self,enabled):
        if enabled and self.session.connected:self.timer.start();self.status()
        else:self.timer.stop()
        self.changed.emit()

    def response(self,op,result,context):
        if not context.startswith('tuning:'):return
        if context=='tuning:profiles':
            self.profiles=result['profiles']
            if self.profile not in self.profiles:self.profile=self.profiles[0] if self.profiles else ''
        elif context.startswith('tuning:keys:'):
            prefix=context.split(':',2)[2]
            requests=[]
            for key in result['items']:
                requests.append(('params.describe',{'key':key},'tuning:load'))
            if result['items']:
                requests.append(('params.get',{'keys':result['items']},'tuning:load'))
            if result.get('next_offset') is not None:
                requests.append(('params.keys',{'prefix':prefix,'offset':result['next_offset'],'limit':8},context))
            self.queue=requests+self.queue
        elif context=='tuning:load':
            if op=='params.describe':self.metas[result['key']]=result
            elif 'values' in result:self.values.update(result['values'])
            else:self.values[result['key']]=result['value']
        elif context=='tuning:save':
            self.values.update(result['values'])
            for key in result['values']:self.drafts.pop(key,None)
            self.notice='Настройки сохранены на роботе.';self.changed.emit();return
        elif context=='tuning:select':
            self.detector=result
            self.notice='Цветовой профиль переключён без перезапуска видео.'
            self.changed.emit();return
        elif context=='tuning:status':
            self.detector=result
        elif context=='tuning:action':
            self.detector=result
            if op=='detection.start':
                self.timer.start()
                self.notice='Проверка запущена. В «Стримах» обновите список и смотрите detection. Используются сохранённые параметры.'
            else:
                self.notice='Проверка остановлена; камера и игровое зрение не отключаются.'
            self.changed.emit();return
        self.next()

    def failed(self,op,error,context):
        if context.startswith('tuning:'):
            self.notice=str(error.get('message',error) if isinstance(error,dict) else error)
            self.busy=False;self.catalog_dirty=False;self.queue=[];self.catalogChanged.emit();self.changed.emit()

    @Slot(str)
    def select(self,profile):
        if profile not in self.profiles:return
        self.profile=profile
        if self.detector.get('running') and self.detector.get('mode')=='colour':
            sent=self.control.command('detection.select',{'profile':profile},manual=False,job=False,context='tuning:select')
            if not sent:self.notice=self.control.error
        self.catalogChanged.emit();self.changed.emit()

    @Slot(str,'QVariant')
    def edit(self,key,value):
        try:
            value=scalar(self.metas[key],value)
            current=self.values|self.drafts
            if key.endswith('_min') and key[:-4]+'_max' in current and value>current[key[:-4]+'_max']:
                raise ValueError('Нижняя граница не должна превышать верхнюю.')
            if key.endswith('_max') and key[:-4]+'_min' in current and value<current[key[:-4]+'_min']:
                raise ValueError('Верхняя граница не должна быть меньше нижней.')
            self.drafts[key]=value
        except (KeyError,ValueError,TypeError) as exc:self.notice=str(exc)
        self.changed.emit()

    @Slot(str)
    def defaults(self,scope):
        for key in self.keys(scope):self.drafts[key]=self.metas[key]['default']
        self.changed.emit()

    @Slot()
    def discard(self):self.drafts={};self.changed.emit()

    @Slot(str)
    def save(self,scope):
        if self.busy:return
        keys=[k for k in self.keys(scope) if k in self.drafts]
        if not keys:return
        self.control.command('params.set',{'values':{k:self.drafts[k] for k in keys},
            'expected_values':{k:self.values[k] for k in keys}},manual=False,job=False,context='tuning:save')

    @Slot(str)
    def selectDetector(self,mode):
        if mode in ('colour','ball','football'):
            self.detector_mode=mode;self.changed.emit()

    @Slot(str)
    def action(self,op):
        if op not in ('detection.start','detection.stop'):return
        if op=='detection.start' and self.detector_mode=='colour' and self.profile not in self.profiles:
            self.notice='Сначала загрузите настройки и выберите цветовой фильтр.';self.changed.emit();return
        args=({'mode':self.detector_mode,'profile':self.profile} if self.detector_mode=='colour'
              else {'mode':self.detector_mode}) if op=='detection.start' else {}
        if op=='detection.start' and self.detector_mode in ('ball','football') and self.control.mode=='GAME':
            self.notice='Остановите игру перед отдельной диагностикой мяча.';self.changed.emit();return
        sent=self.control.command(op,args,manual=op.endswith('.start') and self.control.mode != 'GAME',job=False,context='tuning:action')
        if not sent:
            self.notice=self.control.error;self.changed.emit()

    def shutdown(self):
        self.timer.stop()
