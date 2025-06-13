#include "FlowLayout.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QIcon>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPixmap>
#include <QProcess>
#include <QPushButton>
#include <QScrollArea>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>
#include <QDesktopServices>
#include <QTextBrowser>
#include <QUrl>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QDir>

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

QString autoLinkUrls(const QString &text)
{
    QRegularExpression re(R"((https?://[^\s<]+))");
    QString html = text;
    html.replace(re, R"(<a href="\1">\1</a>)");
    html.replace("\n", "<br>");
    return html;
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
        // Set minimum size so only one demo cell fits in a row
        setMinimumWidth(200); // Adjust to your cell width + margins
        setMinimumHeight(300); // Reasonable minimum height

        // Search field
        searchEdit = new QLineEdit(this);
        searchEdit->setPlaceholderText("Search...");
        mainLayout->addWidget(searchEdit);
        // Make the demo area scrollable if not all fit vertically
        QScrollArea *scrollArea = new QScrollArea(this);
        scrollArea->setWidgetResizable(true);

        QWidget *flowWidget = new QWidget(this);
        flowLayout = new FlowLayout(flowWidget, 10, 10, 10);
        flowWidget->setLayout(flowLayout);

        scrollArea->setWidget(flowWidget);
        mainLayout->addWidget(scrollArea);

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
        struct DemoEntry
        {
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
                else if (std::any_of(tags.begin(), tags.end(), [&](const QString &tag)
                                     { return containsCI(tag, filter); }))
                    rank = 1;
                else if (containsCI(description, filter))
                    rank = 2;

                if (rank < 3)
                    filtered.push_back(DemoEntry{&demo, rank});
            }
        }

        // Sort by match rank, then by headline
        std::sort(filtered.begin(), filtered.end(), [](const DemoEntry &a, const DemoEntry &b)
                  {
            if (a.matchRank != b.matchRank)
                return a.matchRank < b.matchRank;
            QString ha = QString::fromStdString(a.demo->value("headline", ""));
            QString hb = QString::fromStdString(b.demo->value("headline", ""));
            return ha.toLower() < hb.toLower(); });

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
            "QPushButton:hover { background: #e0eaff; border: 1px solid #3399ff; border-radius: 6px; }");

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

        // Description browser (default, supports links)
        QTextBrowser *descBrowser = new QTextBrowser(cellWidget);
        descBrowser->setOpenExternalLinks(true);
        descBrowser->setHtml(description);
        descBrowser->setVisible(false);
        descBrowser->setMinimumHeight(60);
        descBrowser->setMaximumHeight(150);

        // Description editor (hidden by default)
        QTextEdit *descEdit = new QTextEdit(cellWidget);
        descEdit->setAcceptRichText(true);
        descEdit->setText(description);
        descEdit->setVisible(false);
        descEdit->setMinimumHeight(60);
        descEdit->setMaximumHeight(150);

        // Edit button to toggle between view and edit
        QPushButton *editBtn = new QPushButton("Edit", cellWidget);
        editBtn->setVisible(false); // Only show when description is visible
        cellLayout->addWidget(editBtn, 0, Qt::AlignHCenter);

        // Scroll area for browser
        QScrollArea *descScroll = new QScrollArea(cellWidget);
        descScroll->setWidget(descBrowser);
        descScroll->setWidgetResizable(true);
        descScroll->setVisible(false);
        descScroll->setMinimumHeight(60);
        descScroll->setMaximumHeight(150);
        descScroll->setFrameShape(QFrame::NoFrame);
        cellLayout->addWidget(descScroll, 0, Qt::AlignHCenter);

        // Scroll area for editor
        QScrollArea *editScroll = new QScrollArea(cellWidget);
        editScroll->setWidget(descEdit);
        editScroll->setWidgetResizable(true);
        editScroll->setVisible(false);
        editScroll->setMinimumHeight(60);
        editScroll->setMaximumHeight(150);
        editScroll->setFrameShape(QFrame::NoFrame);
        cellLayout->addWidget(editScroll, 0, Qt::AlignHCenter);

        // Toggle description on headline click
        connect(headlineBtn, &QPushButton::clicked, [descScroll, editBtn, descBrowser, editScroll, descEdit]()
                {
    bool show = !descScroll->isVisible() && !editScroll->isVisible();
    descScroll->setVisible(show);
    editBtn->setVisible(show);
    descBrowser->setVisible(show);
    if (!show) {
        editScroll->setVisible(false);
        descEdit->setVisible(false);
        descBrowser->setVisible(false);
        descScroll->setVisible(false);
        editBtn->setVisible(false);
    } });

        // Toggle between browser and editor
        connect(editBtn, &QPushButton::clicked, [=]() mutable
                {
    bool editing = editScroll->isVisible();
    if (!editing) {
        // Switch to edit mode
        descEdit->setText(descBrowser->toHtml());
        descScroll->setVisible(false);
        descBrowser->setVisible(false);
        editScroll->setVisible(true);
        descEdit->setVisible(true);
        editBtn->setText("Save");
    } else {
        // Save and switch back to browser mode
        QString editedText = descEdit->toPlainText();
        QString htmlText = autoLinkUrls(editedText);
        descBrowser->setHtml(htmlText);
        editScroll->setVisible(false);
        descEdit->setVisible(false);
        descScroll->setVisible(true);
        descBrowser->setVisible(true);
        editBtn->setText("Edit");

        // --- Save changes to myDemos.json ---
        // Find the demo in demos_ and update its description
        for (auto &demo : demos_) {
            if (QString::fromStdString(demo.value("headline", "")) == headline) {
                demo["description"] = htmlText.toStdString();
                break;
            }
        }
        // Write to myDemos.json in the same directory as demos.json
        QFileInfo fi(HLRS_DEMO_PATH + "/src/");
        QString myDemosPath = fi.dir().filePath("myDemos.json");
        std::ofstream out(myDemosPath.toStdString());
        if (out) {
            out << demos_.dump(2);
            out.close();
        }
    } });

        // Toggle description on headline click
        connect(headlineBtn, &QPushButton::clicked, [descEdit]()
                { descEdit->setVisible(!descEdit->isVisible()); });

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
            } });
        return cellWidget;
    }
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QString demosPath = HLRS_DEMO_PATH + "/src/myDemos.json";
    if (!QFile::exists(demosPath))
        demosPath = HLRS_DEMO_PATH + "/src/demos.json";

    std::cerr << "Reading demos from: " << demosPath.toStdString() << std::endl;
    json demos = readDemosJson(demosPath);

    DemoWindow window(demos);
    window.setWindowTitle("HLRS Demo Launcher");
    window.resize(900, 700);
    window.show();
    return app.exec();
}