#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QtCharts/QXYSeries>
#include <QLineSeries>
#include <QtCharts/QValueAxis>
#include <limits>
#include <QChart>
#include <QChartView>
#include <QInputDialog>
#include <QLabel>
#include <fstream>
#include "rclcpp/time.hpp"

Subscriber::Subscriber(MainWindow *wnd) : Node("robot_monitor_subscriber"), mainWnd(wnd), start_time(std::chrono::nanoseconds(this->now().nanoseconds()))
{

    auto topic_callback =
        [this](sensor_msgs::msg::JointState::SharedPtr msg) -> void
    {
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", 1);
        rclcpp::Time time = msg->header.stamp;
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> msg_time(std::chrono::nanoseconds(time.nanoseconds()));
        mainWnd->pushMessage(std::chrono::duration<double>(msg_time - start_time).count(), msg);
    };
    subscription_ =
        this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, topic_callback);
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), node(new Subscriber(this))
{
    ui->setupUi(this);
    QString titles[] = {"position", "velocity", "effort", "acc"};
    ui->gridLayout->setContentsMargins(0, 0, 0, 0);
    for (int i = 0; i < 4; i++)
    {
        widgets[i] = new Widget(titles[i], this);
        ui->gridLayout->addWidget(widgets[i], i / 2, i % 2);
        for (int j = 0; j < 7; j++)
            m_buffer[i][j].reserve(sampleCount);
    }
    message = new QLabel(this);
    statusBar()->addPermanentWidget(message);
    isScaling = true;
    isLogging = false;
    frames.reserve(sampleCount * 5);
    setMinimumSize(800, 600);
    connect(ui->actionClear, &QAction::triggered, this, &MainWindow::clear);
    connect(ui->actionJoint_1, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_2, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_3, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_4, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_5, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_6, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_7, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionAuto_Scaling, &QAction::triggered, this, &MainWindow::setScaling);
    connect(ui->actionWindow_Width, &QAction::triggered, this, &MainWindow::setWindowWidth);
    connect(ui->actionLine_Width, &QAction::triggered, this, &MainWindow::setLineWidth);
    connect(ui->actionZoom_In, &QAction::triggered, this, &MainWindow::zoomIn);
    connect(ui->actionZoom_Out, &QAction::triggered, this, &MainWindow::zoomOut);
    connect(ui->actionZoom_Reset, &QAction::triggered, this, &MainWindow::zoomReset);
    connect(ui->actionLoging, &QAction::triggered, this, &MainWindow::setLogging);
    connect(ui->actionSave_Data, &QAction::triggered, this, &MainWindow::log2file);
    connect(&timer, &QTimer::timeout, this, &MainWindow::onTimer);

    tt = std::make_shared<std::thread>([this]()
                                       { 

                                        auto executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
                                        executor->add_node(node);
                                        executor->spin(); });
    tt->detach();
    timer.start(50);
}
void MainWindow::log2file()
{
    std::ofstream fout("log.txt");
    fout.setf(std::ios::fixed);
    fout.precision(9);
    std::lock_guard<std::mutex> gard(mtx);
    for (std::size_t k = 0; k < frames.size(); k++)
    {
        fout << frames[k].t << "\n";
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 7; j++)
                fout << frames[k].q[7 * i + j] << (j == 6 ? "\n" : " ");

        fout << "\n";
    }
}
void MainWindow::setWindowWidth()
{
    bool ok;
    double ww = QInputDialog::getDouble(this, tr("Input Window Width"),
                                        tr("Window Width:"), 10, 0, 100, 2, &ok);
    if (ok)
    {
        time_width = ww;
    }
}
void MainWindow::setLineWidth()
{
    bool ok;
    double ww = QInputDialog::getDouble(this, tr("Input Line Width"),
                                        tr("Line Width:"), 2, 0, 10, 2, &ok);
    if (ok)
    {
        for (int i = 0; i < 4; i++)
            widgets[i]->setLineWidth(ww);
    }
}

void MainWindow::zoomIn()
{
    for (int i = 0; i < 4; i++)
        widgets[i]->getChart()->zoomIn();
}
void MainWindow::zoomOut()
{
    for (int i = 0; i < 4; i++)
        widgets[i]->getChart()->zoomOut();
}
void MainWindow::zoomReset()
{
    for (int i = 0; i < 4; i++)
        widgets[i]->getChart()->zoomReset();
}

void MainWindow::setScaling(bool checked)
{
    isScaling = checked;
}

void MainWindow::setLogging(bool checked)
{
    isLogging = checked;
    if (isLogging)
        message->setText("logging data");
    else
        message->setText("");
}

void MainWindow::setJointDisplay()
{
    int state[] = {ui->actionJoint_1->isChecked(), ui->actionJoint_2->isChecked(), ui->actionJoint_3->isChecked(),
                   ui->actionJoint_4->isChecked(), ui->actionJoint_5->isChecked(), ui->actionJoint_6->isChecked(),
                   ui->actionJoint_7->isChecked()};
    QLineSeries **series[] = {widgets[0]->getSeries(), widgets[1]->getSeries(), widgets[2]->getSeries(), widgets[3]->getSeries()};
    for (int i = 0; i < 7; i++)
        for (int m = 0; m < 4; m++)
        {
            series[m][i]->setVisible(state[i]);
        }
}

void MainWindow::onTimer()
{
    QLineSeries **series[] = {widgets[0]->getSeries(), widgets[1]->getSeries(), widgets[2]->getSeries(), widgets[3]->getSeries()};

    while (!mtx.try_lock())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    if (m_buffer[0][0].size() > 0)
    {
        double t0 = m_buffer[0][0].first().x(), t1 = m_buffer[0][0].back().x();
        for (int m = 0; m < 4; m++)
        {
            std::vector<double> ls, hs;
            for (int i = 0; i < 7; i++)
            {
                series[m][i]->replace(m_buffer[m][i]);
                auto min_max = std::minmax_element(m_buffer[m][i].begin(), m_buffer[m][i].end(),
                                                   [](const QPointF &l, const QPointF &r)
                                                   { return l.y() < r.y(); });
                ls.push_back(min_max.first->y());
                hs.push_back(min_max.second->y());
            }
            if (isScaling)
            {
                double ymin = *std::min_element(ls.begin(), ls.end()), ymax = *std::max_element(hs.begin(), hs.end());
                if (ymax - ymin < 1e-9)
                {
                    ymin = ymax - 1;
                    ymax = ymax + 1;
                }
                widgets[m]->getYAxis()->setRange(ymin, ymax);
            }
            if (t1 - t0 <= time_width)
                widgets[m]->getXAxis()->setRange(t0, t0 + time_width);
            else
                widgets[m]->getXAxis()->setRange(t1 - time_width, t1);
        }
    }

    mtx.unlock();
}

MainWindow::~MainWindow()
{
    // log2file();
    delete ui;
}

void MainWindow::pushMessage(double t, sensor_msgs::msg::JointState::SharedPtr msg)
{
    static Msg last_pack;
    static bool flag = 0;
    static double last_t = 0;
    // update joint state message and dynamic joint state message
    int n = msg->name.size();
    Msg pack;
    pack.t = t;
    // double q[28] = {0};
    for (int i = 0; i < 7; i++)
        i < n && !std::isnan(msg->position[i]) ? pack.q[i] = msg->position[i] : pack.q[i];
    for (int i = 0; i < 7 && msg->velocity.size() > 0; i++)
        i < n && !std::isnan(msg->velocity[i]) ? pack.q[i + 7] = msg->velocity[i] : pack.q[i];
    for (int i = 0; i < 7 && msg->effort.size() > 0; i++)
        i < n && !std::isnan(msg->effort[i]) ? pack.q[i + 14] = msg->effort[i] : pack.q[i];

    if (flag)
    {
        for (int i = 0; i < 7; i++)
            pack.q[21 + i] = (pack.q[7 + i] - last_pack.q[7 + i]) / (t - last_t);
    }
    else
    {
        flag = 1;
    }
    last_t = t;
    last_pack = pack;

    while (!mtx.try_lock())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    if (m_buffer[0][0].size() < sampleCount)
    {
        for (int m = 0; m < 4; m++)
        {
            for (int i = 0; i < 7; i++)
            {
                m_buffer[m][i].append(QPointF(t, pack.q[m * 7 + i]));
            }
        }
    }
    else
    {
        for (int m = 0; m < 4; m++)
        {
            for (int i = 0; i < 7; i++)
                m_buffer[m][i].removeFirst(), m_buffer[m][i].append(QPointF(t, pack.q[m * 7 + i]));
        }
    }
    if (isLogging)
        frames.push_back(pack);

    mtx.unlock();
}

double MainWindow::getDataDuration()
{
    std::lock_guard<std::mutex> gard(mtx);
    return m_buffer[0][0].back().x() - m_buffer[0][0].first().x();
}

void MainWindow::clear()
{
    std::lock_guard<std::mutex> gard(mtx);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            m_buffer[i][j].clear();
        }
    }
    frames.clear();
}
