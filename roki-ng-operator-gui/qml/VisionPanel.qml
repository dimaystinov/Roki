import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ScrollView {
    id: root
    property var kddockwidgets_min_size:Qt.size(480,400)
    leftPadding:8; rightPadding:16; topPadding:6; bottomPadding:6
    ScrollBar.vertical: ScrollBar { parent:root; anchors.right:parent.right; anchors.top:parent.top; anchors.bottom:parent.bottom }
    property var reception: { streams.view; return streams.reception("detection") }
    property var streamDetail: { streams.view; return streams.detail("detection") }
    property bool subscribed: streamDetail.subscribed === true || ["inspecting","preparing","joining","reconciling","receiving"].includes(reception.phase)
    property bool autoStatus:false
    contentWidth:availableWidth
    clip:true
    property var names:({orange_ball:"Оранжевый мяч",green_field:"Зелёное поле",white_marking:"Белая разметка",blue_posts:"Синие стойки",yellow_posts:"Жёлтые стойки",white_posts:"Белые стойки"})
    property var labels:({l_min:"L минимум",l_max:"L максимум",a_min:"a минимум",a_max:"a максимум",b_min:"b минимум",b_max:"b максимум",pixels_min:"Минимум пикселей",box_area_min:"Минимум площади рамки"})
    property string diagnosticFrame: ""
    property string profilesSignature:JSON.stringify(visionTuning.view.profiles)
    property var profileChoices:JSON.parse(profilesSignature).map(k => ({key:k,label:names[k] || k}))
    function updateDiagnosticFrame() { diagnosticFrame = streams.imageUrl("detection") }
    Connections {
        target: streams
        function onFramesChanged() { root.updateDiagnosticFrame() }
        function onChanged() { root.updateDiagnosticFrame() }
    }
    onVisibleChanged: visionTuning.watch(visible && autoStatus)
    Component.onDestruction: visionTuning.watch(false)
    ColumnLayout {
        width:root.availableWidth
        spacing:6
        RowLayout {
            Layout.fillWidth:true
            Button {text:"Загрузить настройки";enabled:backend.view.connected && !visionTuning.view.busy;onClicked:visionTuning.refresh()}
            Button {text:"Обновить статус";enabled:backend.view.connected && !visionTuning.view.busy;onClicked:visionTuning.status()}
            Item {Layout.fillWidth:true}
            ToolButton {text:"?";onClicked:help.open()}
        }
        RowLayout {
            Layout.fillWidth:true
            Label {text:"Проверка:"}
            ComboBox {
                objectName:"tuningDetectorMode";Layout.fillWidth:true
                model:["Цветовая маска","Мяч","Мяч и ворота"]
                enabled:!visionTuning.view.detector.running && !controls.view.pending
                currentIndex:visionTuning.view.detectorMode === "football" ? 2 : visionTuning.view.detectorMode === "ball" ? 1 : 0
                onActivated:visionTuning.selectDetector(currentIndex === 2 ? "football" : currentIndex === 1 ? "ball" : "colour")
            }
            Button {
                objectName:"tuningDetectorStart"
                text:visionTuning.view.detector.running ? "Остановить" : "Запустить"
                enabled:controls.view.owns && !controls.view.pending && (visionTuning.view.detector.running || ((controls.view.manual || (controls.view.mode === "GAME" && visionTuning.view.detectorMode === "colour")) && (visionTuning.view.detectorMode !== "colour" || profile.count>0)))
                onClicked:visionTuning.action(visionTuning.view.detector.running ? "detection.stop" : "detection.start")
            }
        }
        RowLayout {
            Layout.fillWidth:true
            Label {text:visionTuning.view.detector.running ? "Работает" : "Остановлен"}
            Label {
                property var r:visionTuning.view.detector.result
                Layout.fillWidth:true;elide:Text.ElideRight
                text:!r || r.frame_sequence == null ? "Нет результата" : visionTuning.view.detector.mode === "colour"
                    ? "Кадр "+r.frame_sequence+" · областей "+r.total_blobs+" · отсев "+r.area_rejected+" · "+visionTuning.view.detector.age_ms+" мс"
                    : r.valid && typeof r.x_m === "number" && typeof r.y_m === "number" ? "X="+r.x_m.toFixed(3)+" · Y="+r.y_m.toFixed(3)+" м" : "Мяч не принят: "+r.reason
                HoverHandler {id:summaryHover}
                ToolTip.visible:summaryHover.hovered;ToolTip.text:text
            }
            ToolButton {text:"⋯";onClicked:details.open();ToolTip.visible:hovered;ToolTip.text:"Подробности детектора"}
        }
        SelectableLabel {text:visionTuning.view.detector.error || visionTuning.view.detector.video?.error || "";visible:text.length>0;color:"#b03030";Layout.fillWidth:true;wrapMode:Text.Wrap}
        RowLayout {
            Layout.fillWidth:true
            Button {
                objectName:"tuningStreamToggle"
                text:root.subscribed ? "Отключить стрим" : "Запросить стрим"
                enabled:backend.view.connected && !streams.view.busyKeys.includes("detection") && !root.reception.stopping
                onClicked:root.subscribed ? streams.unsubscribe("detection") : streams.watchEmbedded("detection")
            }
            Label {
                Layout.fillWidth:true;wrapMode:Text.Wrap
                text:root.reception.error || (root.reception.active ? (root.reception.size || "Ожидание кадра")+" · "+(root.reception.fps == null ? "—" : root.reception.fps.toFixed(1))+" FPS"+(root.reception.rtpText ? " · "+root.reception.rtpText : "") : root.subscribed ? "Ожидание кадра…" : "Стрим не запрошен")
            }
        }
        Label {visible:root.reception.stalled || false;text:"Нет новых кадров. Проверьте передачу и UDP.";color:"#b03030";Layout.fillWidth:true;wrapMode:Text.Wrap}
        Image {
            objectName:"tuningRobotVideo"
            visible:source.toString() !== "" && status === Image.Ready
            Layout.fillWidth:true
            Layout.preferredHeight:visible && sourceSize.width>0 ? width*sourceSize.height/sourceSize.width : 0
            fillMode:Image.PreserveAspectFit;cache:false;source:root.diagnosticFrame
        }
        ColumnLayout {
            Layout.fillWidth:true;spacing:6
            ComboBox {
                id:profile;Layout.fillWidth:true
                enabled:!visionTuning.view.loading && !controls.view.pending
                model:root.profileChoices;textRole:"label"
                currentIndex:visionTuning.view.profiles.indexOf(visionTuning.view.profile)
                displayText:count ? currentText : "Загрузите настройки цветов"
                onActivated:visionTuning.select(visionTuning.view.profiles[currentIndex])
            }
            Repeater {
                model:["l", "a", "b"]
                RowLayout {
                    id:axisRow
                    required property string modelData
                    property string prefix:"vision."+visionTuning.view.profile+"."+modelData
                    property var lowMeta:visionTuning.view.metas[prefix+"_min"] || {}
                    property var highMeta:visionTuning.view.metas[prefix+"_max"] || {}
                    property real low:visionTuning.view.values[prefix+"_min"] ?? 0
                    property real high:visionTuning.view.values[prefix+"_max"] ?? 0
                    Layout.fillWidth:true
                    visible:visionTuning.view.profile !== ""
                    Label {text:axisRow.modelData === "l" ? "L" : axisRow.modelData;Layout.preferredWidth:25}
                    ValueEditor {
                        objectName:"tuningEditor-"+axisRow.modelData+"_min"
                        Layout.preferredWidth:80;commitOnFinish:true
                        meta:axisRow.lowMeta;initialValue:axisRow.low
                        onEdited:value => visionTuning.edit(axisRow.prefix+"_min",value)
                    }
                    RangeSlider {
                        id:axisRange
                        objectName:"tuningRange-"+axisRow.modelData
                        Layout.fillWidth:true
                        from:axisRow.lowMeta.min ?? 0;to:axisRow.highMeta.max ?? 100;stepSize:1
                        function syncValues() { if (!first.pressed && !second.pressed) setValues(axisRow.low,axisRow.high) }
                        Component.onCompleted:Qt.callLater(syncValues)
                        Connections {
                            target:axisRow
                            function onLowChanged() { Qt.callLater(axisRange.syncValues) }
                            function onHighChanged() { Qt.callLater(axisRange.syncValues) }
                        }
                        first.onMoved:visionTuning.edit(axisRow.prefix+"_min",Math.round(first.value))
                        second.onMoved:visionTuning.edit(axisRow.prefix+"_max",Math.round(second.value))
                    }
                    ValueEditor {
                        objectName:"tuningEditor-"+axisRow.modelData+"_max"
                        Layout.preferredWidth:80;commitOnFinish:true
                        meta:axisRow.highMeta;initialValue:axisRow.high
                        onEdited:value => visionTuning.edit(axisRow.prefix+"_max",value)
                    }
                }
            }
            ToolButton {id:areaToggle;checkable:true;text:(checked ? "▾" : "▸")+" Фильтрация по площади"}
            Repeater {
                model:["pixels_min", "box_area_min"]
                RowLayout {
                    id:areaRow
                    required property string modelData
                    property string key:"vision."+visionTuning.view.profile+"."+modelData
                    Layout.fillWidth:true;visible:areaToggle.checked && visionTuning.view.profile !== ""
                    Label {text:root.labels[areaRow.modelData];Layout.fillWidth:true}
                    ValueEditor {
                        objectName: "tuningEditor-" + areaRow.modelData
                        commitOnFinish: true
                        Layout.preferredWidth:95
                        meta:visionTuning.view.metas[areaRow.key] || {};initialValue:visionTuning.view.values[areaRow.key]
                        onEdited:value => visionTuning.edit(areaRow.key,value)
                    }
                }
            }
            RowLayout {
                Layout.fillWidth:true
                Button {id:saveThresholds;text:"Сохранить";ToolTip.visible:hovered;ToolTip.text:"Сохранить пороги (пробел)";enabled:visionTuning.view.dirty && controls.view.owns && !controls.view.pending && !visionTuning.view.busy;onClicked:visionTuning.save("lab")}
                Shortcut {
                    sequence: "Space"
                    autoRepeat: false
                    enabled: root.visible && root.activeFocus && saveThresholds.enabled
                             && !(controls.view.manual && controls.view.keyboard)
                    onActivated: saveThresholds.clicked()
                }
                Button {text:"Отменить";enabled:visionTuning.view.dirty && !visionTuning.view.busy;onClicked:visionTuning.discard();ToolTip.visible:hovered;ToolTip.text:"Отменить черновики всех цветов"}
                Item {Layout.fillWidth:true}
                Button {text:"Сбросить…";enabled:!visionTuning.view.busy && profile.count>0;onClicked:reset.open()}
            }
        }
        Label {visible:visionTuning.view.dirty;text:"Есть несохранённые изменения";color:"#bd7b17"}
        SelectableLabel {text:visionTuning.view.notice;visible:text.length>0;Layout.fillWidth:true;wrapMode:Text.Wrap}
        SelectableLabel {text:controls.view.error || streams.view.error;visible:text.length>0;color:"#b03030";Layout.fillWidth:true;wrapMode:Text.Wrap}
    }
    Dialog {
        id:reset;title:"Вернуть стандартный фильтр?"
        anchors.centerIn:parent;modal:true;standardButtons:Dialog.Ok | Dialog.Cancel
        onAccepted:visionTuning.defaults("lab")
    }
    Dialog {
        id:details;title:"Подробности детектора"
        anchors.centerIn:parent;width:Math.min(root.width-16,580);height:Math.min(root.height-16,420)
        standardButtons:Dialog.Close
        ScrollView {
            anchors.fill:parent;contentWidth:availableWidth;clip:true
            ColumnLayout {
                width:parent.width
                CheckBox {text:"Автообновление статуса";checked:root.autoStatus;onToggled:{root.autoStatus=checked;visionTuning.watch(checked && root.visible)}}
                RawDetails {text:JSON.stringify(visionTuning.view.detector,null,2);Layout.fillWidth:true}
            }
        }
    }
    Dialog {
        id:help;title:"Цвета и детекция"
        anchors.centerIn:parent;width:Math.min(root.width-16,580);height:Math.min(root.height-16,480)
        standardButtons:Dialog.Close
        ScrollView {
            anchors.fill:parent;contentWidth:availableWidth;clip:true
            Label {
                width:parent.width;wrapMode:Text.Wrap
                text:"Загрузите настройки, выберите объект, задайте L, a, b и сохраните. До сохранения значения — черновик. При работающей цветовой маске выбор объекта переключает фильтр без перезапуска стрима.\n\nМинимум пикселей — площадь цветной области; минимум площади рамки — ширина × высота рамки.\n\nЦветовая маска показывает совпадение с порогами, а не подтверждение мяча. Мяч: игровой детектор, принятые и отклонённые кандидаты. selected — выбран; no_field — нет опоры поля/разметки; projection — луч вне калибровки или над горизонтом; range — вне дальности; area_rejected — отсев по площади.\n\nМяч: X вперёд, Y влево относительно направления головы; результат статуса — в метрах. Мяч и ворота: на видео X вперёд, Y влево относительно корпуса, D — горизонтальная дальность; единицы — миллиметры. H — расчётная высота камеры. Оранжевая рамка colour — цветной ориентир, зелёная opening — проём ворот.\n\nДля расстояний нужны калибровка камеры, свежая IMU и соответствие реальной высоты камеры расчётной. Проверка «Мяч и ворота» требует неподвижного подключённого тела, но не метки base_stand. После движения тела или головы перезапустите проверку. Она сама не двигает робот.\n\nКамера и ISP настраиваются в «Камере». «Запросить стрим» принимает detection внутри панели без внешнего окна. «Отключить стрим» снимает подписку этого GUI на роботе и прекращает приём во всех его просмотрах detection; другие клиенты не отключаются."
            }
        }
    }
}
