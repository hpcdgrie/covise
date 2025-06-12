#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QDir>
#include <QMessageBox>
#include <QFile>
#include <QProcess>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <fstream>
#include <nlohmann/json.hpp>
#include <iostream>
#include <QIcon>
#include <QPixmap>
#include <QLineEdit>
#include <algorithm>
#include "FlowLayout.h"

using json = nlohmann::json;

// Helper function to read demos.json using nlohmann::json
json readDemosJson(const QString &path)
{
    std::ifstream file(path.toStdString());
    if (!file)
        return {};
    json j;
    file >> j;
    return j;
}

// Normalize path for cross-platform compatibility
const QString COVISE_PATH = QDir::cleanPath(getenv("COVISE_PATH") ? getenv("COVISE_PATH") : "");
const QString HLRS_DEMO_PATH = COVISE_PATH + "/src/tools/hlrsDemo";

class DemoWindow : public QWidget
{
public:
    DemoWindow(const json &demos, QWidget *parent = nullptr)
        : QWidget(parent), demos_(demos)
    {
        QVBoxLayout *mainLayout = new QVBoxLayout(this);

        // Search field
        searchEdit = new QLineEdit(this);
        searchEdit->setPlaceholderText("Search...");
        mainLayout->addWidget(searchEdit);

        // Flow layout in a child widget
        QWidget *flowWidget = new QWidget(this);
        flowLayout = new FlowLayout(flowWidget, 10, 10, 10);
        flowWidget->setLayout(flowLayout);
        mainLayout->addWidget(flowWidget);

        setLayout(mainLayout);

        connect(searchEdit, &QLineEdit::textChanged, this, &DemoWindow::updateSearch);

        createCells();
    }

protected:
    void resizeEvent(QResizeEvent *event) override
    {
        QWidget::resizeEvent(event);
        // FlowLayout handles reflow automatically
    }

private:
    json demos_;
    FlowLayout *flowLayout;
    std::vector<QWidget *> cellWidgets;
    QLineEdit *searchEdit;

    // Helper: returns lowercase version of a QString
    static QString lower(const QString &s) { return s.toLower(); }

    // Helper: returns true if haystack contains needle (case-insensitive)
    static bool containsCI(const QString &haystack, const QString &needle)
    {
        return haystack.toLower().contains(needle.toLower());
    }

    void createCells(const QString &filter = QString())
    {
        // Remove old widgets if any
        for (auto *w : cellWidgets)
            delete w;
        cellWidgets.clear();

        // Remove all widgets from layout
        QLayoutItem *item;
        while ((item = flowLayout->takeAt(0)) != nullptr)
        {
            if (item->widget())
                item->widget()->setParent(nullptr);
            delete item;
        }

        // Prepare filtered and sorted demo list
        struct DemoEntry {
            const json *demo;
            int matchRank; // 0 = title, 1 = tags, 2 = description, 3 = no match
        };
        std::vector<DemoEntry> filtered;

        if (demos_.is_array())
        {
            for (const auto &demo : demos_)
            {
                QString headline = QString::fromStdString(demo.value("headline", "Unnamed"));
                QString description = QString::fromStdString(demo.value("description", ""));
                QStringList tags;
                if (demo.contains("tags") && demo["tags"].is_array())
                {
                    for (const auto &tag : demo["tags"])
                        tags << QString::fromStdString(tag.get<std::string>());
                }

                int rank = 3;
                if (filter.isEmpty())
                    rank = 0;
                else if (containsCI(headline, filter))
                    rank = 0;
                else if (std::any_of(tags.begin(), tags.end(), [&](const QString &tag){ return containsCI(tag, filter); }))
                    rank = 1;
                else if (containsCI(description, filter))
                    rank = 2;

                if (rank < 3)
                    filtered.push_back(DemoEntry{&demo, rank});
            }
        }

        // Sort by match rank, then by headline
        std::sort(filtered.begin(), filtered.end(), [](const DemoEntry &a, const DemoEntry &b) {
            if (a.matchRank != b.matchRank)
                return a.matchRank < b.matchRank;
            QString ha = QString::fromStdString(a.demo->value("headline", ""));
            QString hb = QString::fromStdString(b.demo->value("headline", ""));
            return ha.toLower() < hb.toLower();
        });

        // Create all cell widgets and store them
        for (const auto &entry : filtered)
        {
            const json &demo = *entry.demo;
            QString headline = QString::fromStdString(demo.value("headline", "Unnamed"));
            QString imagePath;
            if (demo.contains("image") && demo["image"].is_string())
            {
                imagePath = HLRS_DEMO_PATH + "/src/screenshots/" + QString::fromStdString(demo["image"]);
            }

            if (demo.contains("launch") && demo["launch"].is_array())
            {
                std::vector<std::pair<QString, QStringList>> programs;
                for (const auto &launch : demo["launch"])
                {
                    QString program = QString::fromStdString(launch.value("program", ""));
                    QStringList args;
                    if (launch.contains("args") && launch["args"].is_array())
                    {
                        for (const auto &arg : launch["args"])
                        {
                            args << QString::fromStdString(arg.get<std::string>());
                        }
                    }
                    programs.push_back({program, args});
                }
                QString description = QString::fromStdString(demo.value("description", ""));

                auto *cellWidget = createDemoWidget(headline, description, imagePath, programs);
                cellWidgets.push_back(cellWidget);
                flowLayout->addWidget(cellWidget);
            }
        }
    }

    void updateSearch(const QString &text)
    {
        createCells(text);
    }

    QWidget *createDemoWidget(const QString &headline, const QString &description, const QString &imagePath,
                              const std::vector<std::pair<QString, QStringList>> &programs)
    {
        QWidget *cellWidget = new QWidget(this);
        QVBoxLayout *cellLayout = new QVBoxLayout(cellWidget);
        cellLayout->setContentsMargins(5, 5, 5, 5);
        cellLayout->setSpacing(5);

        // Create a button with the image
        QPushButton *btn = new QPushButton(cellWidget);
        btn->setFixedSize(140, 140);
        btn->setIconSize(QSize(128, 128));
        btn->setText("");

        // Remove button border and background
        btn->setStyleSheet(
            "QPushButton { border: none; background: transparent; }"
            "QPushButton:hover { background: #e0eaff; border: 1px solid #3399ff; border-radius: 6px; }"
        );

        if (!imagePath.isEmpty() && QFile::exists(imagePath))
        {
            QPixmap pix(imagePath);
            btn->setIcon(QIcon(pix.scaled(128, 128, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
        }
        cellLayout->addWidget(btn, 0, Qt::AlignHCenter);

        // Headline as clickable label
        QPushButton *headlineBtn = new QPushButton(headline, cellWidget);
        headlineBtn->setFlat(true);
        headlineBtn->setStyleSheet("QPushButton { color: blue; text-decoration: underline; background: transparent; border: none; }");
        headlineBtn->setCursor(Qt::PointingHandCursor);
        cellLayout->addWidget(headlineBtn, 0, Qt::AlignHCenter);

        // Description label, initially hidden
        QLabel *descLabel = new QLabel(description, cellWidget);
        descLabel->setWordWrap(true);
        descLabel->setAlignment(Qt::AlignCenter);
        descLabel->setVisible(false);
        cellLayout->addWidget(descLabel, 0, Qt::AlignHCenter);

        // Toggle description on headline click
        connect(headlineBtn, &QPushButton::clicked, [descLabel]()
                { descLabel->setVisible(!descLabel->isVisible()); });

        // Launch application on image click
        connect(btn, &QPushButton::clicked, [programs]()
                {
            for (const auto &[program, args] : programs)
            {
                if (program.isEmpty()) {
                    QMessageBox::warning(nullptr, "Error", "No program specified for this demo.");
                    return;
                }
                bool started = QProcess::startDetached(program, args);
                if (!started) {
                    QMessageBox::critical(nullptr, "Error", "Failed to start: " + program);
                } 
            } 
        });
        return cellWidget;
    }
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QString demosPath = HLRS_DEMO_PATH + "/src/demos.json";
    std::cerr << "Reading demos from: " << demosPath.toStdString() << std::endl;
    json demos = readDemosJson(demosPath);

    DemoWindow window(demos);
    window.setWindowTitle("HLRS Demo Launcher");
    window.resize(900, 700);
    window.show();
    return app.exec();
}