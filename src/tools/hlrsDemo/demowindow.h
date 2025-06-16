#ifndef HLRS_DEMO_WINDOW_H
#define HLRS_DEMO_WINDOW_H

#include "flowlayout.h"

#include <QWidget>
#include <QResizeEvent>
#include <QLineEdit>
#include <QString>
#include <QProcess>
#include <nlohmann/json.hpp>

#include <map>
#include <vector>

extern const QString COVISE_PATH;
extern const QString HLRS_DEMO_PATH;

nlohmann::json readDemosJson(const QString &path);


class DemoWindow : public QWidget
{
public:
    DemoWindow(const nlohmann::json &demos, QWidget *parent = nullptr);
        
protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    nlohmann::json demos_;
    FlowLayout *flowLayout;
    std::vector<QWidget *> cellWidgets;
    QLineEdit *searchEdit;
    std::map<qint64, QProcess *> runningProcesses;             // Track running processes and their PIDs
    std::map<QWidget *, std::vector<qint64>> cellWidgetToPids; // Map cell widget to running PIDs

    void createCells(const QString &filter = QString());
    void updateSearch(const QString &text);
    QWidget *createDemoWidget(const QString &headline, const QString &description, const QString &imagePath,
                              const std::vector<std::pair<QString, QStringList>> &programs);
    
};


#endif // HLRS_DEMO_WINDOW_H