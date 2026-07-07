#pragma once

#include <QApplication>
#include <QCloseEvent>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QShortcut>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <table_top_manip/manip_server.h>

class TeleopGui : public QWidget {
    Q_OBJECT

public:
    explicit TeleopGui(ManipServer& server, QWidget* parent = nullptr)
        : QWidget(parent), _server(server) {
        setWindowTitle("Teleop Data Collection");
        setMinimumWidth(360);
        setFocusPolicy(Qt::StrongFocus);

        auto* root = new QVBoxLayout(this);
        root->setSpacing(8);
        root->setContentsMargins(12, 12, 12, 12);

        // ── Status ────────────────────────────────────────────────────────
        auto* sg = new QGroupBox("Status", this);
        auto* sl = new QVBoxLayout(sg);
        _teleop_lbl    = addStatusRow(sl, "Teleop");
        _recording_lbl = addStatusRow(sl, "Recording");
        _amplify_lbl   = addStatusRow(sl, "Amplify");
        _idle_lbl      = addStatusRow(sl, "Idle mode");

        // ── Controls ──────────────────────────────────────────────────────
        auto* cg = new QGroupBox("Controls", this);
        auto* cl = new QVBoxLayout(cg);
        addCtrlRow(cl, "Enter",        "Start teleop");
        addCtrlRow(cl, "Btn1 + Btn2",  "Stop teleop");
        addCtrlRow(cl, "Btn1 (held)",  "Amplify mode");
        addCtrlRow(cl, "Btn2 (held)",  "Idle mode");
        addCtrlRow(cl, "r",            "Toggle recording");
        addCtrlRow(cl, "q",            "Quit");

        root->addWidget(sg);
        root->addWidget(cg);

        // JY: ApplicationShortcut fires as long as the Qt app is in the foreground,
        //     regardless of which widget has keyboard focus.
        auto makeShortcut = [&](Qt::Key key, auto slot) {
            auto* sc = new QShortcut(QKeySequence(key), this);
            sc->setContext(Qt::ApplicationShortcut);
            connect(sc, &QShortcut::activated, this, slot);
        };
        makeShortcut(Qt::Key_Return, &TeleopGui::onStartTeleop);
        makeShortcut(Qt::Key_Enter,  &TeleopGui::onStartTeleop);
        makeShortcut(Qt::Key_R,      &TeleopGui::onToggleRecording);
        makeShortcut(Qt::Key_Q,      &TeleopGui::onQuit);

        _timer = new QTimer(this);
        connect(_timer, &QTimer::timeout, this, &TeleopGui::updateStatus);
        _timer->start(100);
        updateStatus();
    }

protected:
    void closeEvent(QCloseEvent* ev) override {
        QApplication::quit();
        ev->accept();
    }

private slots:
    void updateStatus() {
        applyStatus(_teleop_lbl,    _server.is_teleop_active(), "ACTIVE",    "INACTIVE");
        applyStatus(_recording_lbl, _server.is_saving_data(),   "RECORDING", "STOPPED");
        applyStatus(_amplify_lbl,   _server.is_amplify_mode(),  "ON",        "OFF");
        applyStatus(_idle_lbl,      _server.is_idle_mode(),     "ON",        "OFF");
    }

    void onStartTeleop()      { _server.request_teleop_start(); }
    void onToggleRecording()  {
        if (_server.is_saving_data()) _server.stop_saving_data();
        else                          _server.start_saving_data_for_a_new_episode();
    }
    void onQuit()             { QApplication::quit(); }

private:
    QLabel* addStatusRow(QVBoxLayout* parent, const QString& name) {
        auto* row  = new QHBoxLayout;
        auto* lnam = new QLabel(name + ":");
        auto* lval = new QLabel;
        lnam->setFixedWidth(90);
        lval->setFixedWidth(110);
        lval->setAlignment(Qt::AlignCenter);
        row->addWidget(lnam);
        row->addWidget(lval);
        row->addStretch();
        parent->addLayout(row);
        return lval;
    }

    void addCtrlRow(QVBoxLayout* parent, const QString& key, const QString& desc) {
        auto* row = new QHBoxLayout;
        auto* lk  = new QLabel(key);
        auto* ld  = new QLabel(desc);
        lk->setFixedWidth(110);
        lk->setStyleSheet("font-family: monospace; font-weight: bold;");
        row->addWidget(lk);
        row->addWidget(ld);
        row->addStretch();
        parent->addLayout(row);
    }

    void applyStatus(QLabel* lbl, bool active, const char* on_txt, const char* off_txt) {
        lbl->setText(active ? on_txt : off_txt);
        lbl->setStyleSheet(active
            ? "font-weight:bold; padding:2px 8px; border-radius:4px;"
              "background:#4caf50; color:white;"
            : "font-weight:bold; padding:2px 8px; border-radius:4px;"
              "background:#9e9e9e; color:white;");
    }

    ManipServer& _server;
    QTimer*      _timer{nullptr};
    QLabel*      _teleop_lbl{nullptr};
    QLabel*      _recording_lbl{nullptr};
    QLabel*      _amplify_lbl{nullptr};
    QLabel*      _idle_lbl{nullptr};
};
