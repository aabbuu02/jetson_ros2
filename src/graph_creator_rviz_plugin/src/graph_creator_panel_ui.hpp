/********************************************************************************
** Form generated from reading UI file 'graph_creator_rviz_plugin.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef GRAPH_CREATOR_PANEL_UI_H
#define GRAPH_CREATOR_PANEL_UI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_graphCreatorPanelForm
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *mainVerticalLayout;
    QTabWidget *mainTabWidget;
    QWidget *graphModelingTab;
    QGridLayout *gridLayout_2;
    QVBoxLayout *graphModelingVerticalLayout;
    QHBoxLayout *modelingModehorizontalLayout;
    QPushButton *createVerticesPushButton;
    QPushButton *createEdgesPushButton;
    QHBoxLayout *edgeCreationTypeHorizontalLayout;
    QLabel *edgeCreationTypelabel;
    QComboBox *edgeCreationTypeComboBox;
    QHBoxLayout *edgeDirectionTypeHorizontalLayout;
    QLabel *edgeDirectionTypeLabel;
    QComboBox *edgeDirectionTypeComboBox;
    QHBoxLayout *lockVerticesEdgesHorizontalLayout;
    QSpacerItem *horizontalSpacer_2;
    QCheckBox *lockVerticesCheckBox;
    QSpacerItem *horizontalSpacer;
    QCheckBox *lockEdgesCheckBox;
    QSpacerItem *horizontalSpacer_3;
    QFrame *line;
    QLabel *graphElementTypeLabel;
    QTableWidget *graphElementTableWidget;
    QWidget *graphFileTab;
    QGridLayout *gridLayout_3;
    QVBoxLayout *graphFileVerticalLayout;
    QHBoxLayout *graphNameHorizontalLayout;
    QLabel *graphNameLabel;
    QSpacerItem *horizontalSpacer_10;
    QLineEdit *graphNameLineEdit;
    QHBoxLayout *horizontalLayout;
    QLabel *graphFileFormatLabel;
    QSpacerItem *horizontalSpacer_11;
    QComboBox *graphFileFormatComboBox;
    QFrame *line_3;
    QHBoxLayout *GraphFileNameHorizontalLayout;
    QLabel *packageNamelabel;
    QSpacerItem *horizontalSpacer_9;
    QLineEdit *packageNameLineEdit;
    QHBoxLayout *graphFolderHorizontalLayout;
    QLabel *graphFolderLabel;
    QSpacerItem *horizontalSpacer_5;
    QLineEdit *graphFolderLineEdit;
    QFrame *line_2;
    QHBoxLayout *graphOperationOnHorizontalLayout;
    QLabel *graphAppOperateOnLabel;
    QSpacerItem *horizontalSpacer_4;
    QComboBox *graphAppTypeComboBox;
    QHBoxLayout *graphFileOperationsHorizontalLayout;
    QSpacerItem *horizontalSpacer_6;
    QPushButton *loadGraphPushButton;
    QSpacerItem *loadSaveHorizontalSpacer;
    QPushButton *saveGraphPushButton;
    QSpacerItem *horizontalSpacer_7;
    QSpacerItem *verticalSpacer;
    QFrame *line_4;
    QTextEdit *graphCreatorInfoTextEdit;
    QFrame *line_5;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer_8;
    QPushButton *refreshButton;
    QSpacerItem *horizontalSpacer_12;

    void setupUi(QWidget *graphCreatorPanelForm)
    {
        if (graphCreatorPanelForm->objectName().isEmpty())
            graphCreatorPanelForm->setObjectName(QStringLiteral("graphCreatorPanelForm"));
        graphCreatorPanelForm->resize(378, 866);
        gridLayout = new QGridLayout(graphCreatorPanelForm);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        mainVerticalLayout = new QVBoxLayout();
        mainVerticalLayout->setObjectName(QStringLiteral("mainVerticalLayout"));
        mainTabWidget = new QTabWidget(graphCreatorPanelForm);
        mainTabWidget->setObjectName(QStringLiteral("mainTabWidget"));
        mainTabWidget->setTabShape(QTabWidget::Rounded);
        graphModelingTab = new QWidget();
        graphModelingTab->setObjectName(QStringLiteral("graphModelingTab"));
        gridLayout_2 = new QGridLayout(graphModelingTab);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        graphModelingVerticalLayout = new QVBoxLayout();
        graphModelingVerticalLayout->setObjectName(QStringLiteral("graphModelingVerticalLayout"));
        modelingModehorizontalLayout = new QHBoxLayout();
        modelingModehorizontalLayout->setObjectName(QStringLiteral("modelingModehorizontalLayout"));
        createVerticesPushButton = new QPushButton(graphModelingTab);
        createVerticesPushButton->setObjectName(QStringLiteral("createVerticesPushButton"));

        modelingModehorizontalLayout->addWidget(createVerticesPushButton);

        createEdgesPushButton = new QPushButton(graphModelingTab);
        createEdgesPushButton->setObjectName(QStringLiteral("createEdgesPushButton"));

        modelingModehorizontalLayout->addWidget(createEdgesPushButton);


        graphModelingVerticalLayout->addLayout(modelingModehorizontalLayout);

        edgeCreationTypeHorizontalLayout = new QHBoxLayout();
        edgeCreationTypeHorizontalLayout->setObjectName(QStringLiteral("edgeCreationTypeHorizontalLayout"));
        edgeCreationTypelabel = new QLabel(graphModelingTab);
        edgeCreationTypelabel->setObjectName(QStringLiteral("edgeCreationTypelabel"));

        edgeCreationTypeHorizontalLayout->addWidget(edgeCreationTypelabel);

        edgeCreationTypeComboBox = new QComboBox(graphModelingTab);
        edgeCreationTypeComboBox->setObjectName(QStringLiteral("edgeCreationTypeComboBox"));

        edgeCreationTypeHorizontalLayout->addWidget(edgeCreationTypeComboBox);


        graphModelingVerticalLayout->addLayout(edgeCreationTypeHorizontalLayout);

        edgeDirectionTypeHorizontalLayout = new QHBoxLayout();
        edgeDirectionTypeHorizontalLayout->setObjectName(QStringLiteral("edgeDirectionTypeHorizontalLayout"));
        edgeDirectionTypeLabel = new QLabel(graphModelingTab);
        edgeDirectionTypeLabel->setObjectName(QStringLiteral("edgeDirectionTypeLabel"));

        edgeDirectionTypeHorizontalLayout->addWidget(edgeDirectionTypeLabel);

        edgeDirectionTypeComboBox = new QComboBox(graphModelingTab);
        edgeDirectionTypeComboBox->setObjectName(QStringLiteral("edgeDirectionTypeComboBox"));

        edgeDirectionTypeHorizontalLayout->addWidget(edgeDirectionTypeComboBox);


        graphModelingVerticalLayout->addLayout(edgeDirectionTypeHorizontalLayout);

        lockVerticesEdgesHorizontalLayout = new QHBoxLayout();
        lockVerticesEdgesHorizontalLayout->setObjectName(QStringLiteral("lockVerticesEdgesHorizontalLayout"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        lockVerticesEdgesHorizontalLayout->addItem(horizontalSpacer_2);

        lockVerticesCheckBox = new QCheckBox(graphModelingTab);
        lockVerticesCheckBox->setObjectName(QStringLiteral("lockVerticesCheckBox"));

        lockVerticesEdgesHorizontalLayout->addWidget(lockVerticesCheckBox);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        lockVerticesEdgesHorizontalLayout->addItem(horizontalSpacer);

        lockEdgesCheckBox = new QCheckBox(graphModelingTab);
        lockEdgesCheckBox->setObjectName(QStringLiteral("lockEdgesCheckBox"));

        lockVerticesEdgesHorizontalLayout->addWidget(lockEdgesCheckBox);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        lockVerticesEdgesHorizontalLayout->addItem(horizontalSpacer_3);


        graphModelingVerticalLayout->addLayout(lockVerticesEdgesHorizontalLayout);

        line = new QFrame(graphModelingTab);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        graphModelingVerticalLayout->addWidget(line);

        graphElementTypeLabel = new QLabel(graphModelingTab);
        graphElementTypeLabel->setObjectName(QStringLiteral("graphElementTypeLabel"));
        graphElementTypeLabel->setFrameShape(QFrame::Box);
        graphElementTypeLabel->setFrameShadow(QFrame::Sunken);
        graphElementTypeLabel->setAlignment(Qt::AlignCenter);

        graphModelingVerticalLayout->addWidget(graphElementTypeLabel);

        graphElementTableWidget = new QTableWidget(graphModelingTab);
        graphElementTableWidget->setObjectName(QStringLiteral("graphElementTableWidget"));
        graphElementTableWidget->setAlternatingRowColors(true);
        graphElementTableWidget->horizontalHeader()->setVisible(true);
        graphElementTableWidget->horizontalHeader()->setStretchLastSection(true);
        graphElementTableWidget->verticalHeader()->setVisible(false);

        graphModelingVerticalLayout->addWidget(graphElementTableWidget);


        gridLayout_2->addLayout(graphModelingVerticalLayout, 0, 0, 1, 1);

        mainTabWidget->addTab(graphModelingTab, QString());
        graphFileTab = new QWidget();
        graphFileTab->setObjectName(QStringLiteral("graphFileTab"));
        gridLayout_3 = new QGridLayout(graphFileTab);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        graphFileVerticalLayout = new QVBoxLayout();
        graphFileVerticalLayout->setObjectName(QStringLiteral("graphFileVerticalLayout"));
        graphNameHorizontalLayout = new QHBoxLayout();
        graphNameHorizontalLayout->setObjectName(QStringLiteral("graphNameHorizontalLayout"));
        graphNameLabel = new QLabel(graphFileTab);
        graphNameLabel->setObjectName(QStringLiteral("graphNameLabel"));

        graphNameHorizontalLayout->addWidget(graphNameLabel);

        horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        graphNameHorizontalLayout->addItem(horizontalSpacer_10);

        graphNameLineEdit = new QLineEdit(graphFileTab);
        graphNameLineEdit->setObjectName(QStringLiteral("graphNameLineEdit"));

        graphNameHorizontalLayout->addWidget(graphNameLineEdit);


        graphFileVerticalLayout->addLayout(graphNameHorizontalLayout);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        graphFileFormatLabel = new QLabel(graphFileTab);
        graphFileFormatLabel->setObjectName(QStringLiteral("graphFileFormatLabel"));

        horizontalLayout->addWidget(graphFileFormatLabel);

        horizontalSpacer_11 = new QSpacerItem(40, 20, QSizePolicy::Maximum, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_11);

        graphFileFormatComboBox = new QComboBox(graphFileTab);
        graphFileFormatComboBox->setObjectName(QStringLiteral("graphFileFormatComboBox"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(graphFileFormatComboBox->sizePolicy().hasHeightForWidth());
        graphFileFormatComboBox->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(graphFileFormatComboBox);


        graphFileVerticalLayout->addLayout(horizontalLayout);

        line_3 = new QFrame(graphFileTab);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);

        graphFileVerticalLayout->addWidget(line_3);

        GraphFileNameHorizontalLayout = new QHBoxLayout();
        GraphFileNameHorizontalLayout->setObjectName(QStringLiteral("GraphFileNameHorizontalLayout"));
        packageNamelabel = new QLabel(graphFileTab);
        packageNamelabel->setObjectName(QStringLiteral("packageNamelabel"));

        GraphFileNameHorizontalLayout->addWidget(packageNamelabel);

        horizontalSpacer_9 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        GraphFileNameHorizontalLayout->addItem(horizontalSpacer_9);

        packageNameLineEdit = new QLineEdit(graphFileTab);
        packageNameLineEdit->setObjectName(QStringLiteral("packageNameLineEdit"));

        GraphFileNameHorizontalLayout->addWidget(packageNameLineEdit);


        graphFileVerticalLayout->addLayout(GraphFileNameHorizontalLayout);

        graphFolderHorizontalLayout = new QHBoxLayout();
        graphFolderHorizontalLayout->setObjectName(QStringLiteral("graphFolderHorizontalLayout"));
        graphFolderLabel = new QLabel(graphFileTab);
        graphFolderLabel->setObjectName(QStringLiteral("graphFolderLabel"));

        graphFolderHorizontalLayout->addWidget(graphFolderLabel);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        graphFolderHorizontalLayout->addItem(horizontalSpacer_5);

        graphFolderLineEdit = new QLineEdit(graphFileTab);
        graphFolderLineEdit->setObjectName(QStringLiteral("graphFolderLineEdit"));

        graphFolderHorizontalLayout->addWidget(graphFolderLineEdit);


        graphFileVerticalLayout->addLayout(graphFolderHorizontalLayout);

        line_2 = new QFrame(graphFileTab);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        graphFileVerticalLayout->addWidget(line_2);

        graphOperationOnHorizontalLayout = new QHBoxLayout();
        graphOperationOnHorizontalLayout->setObjectName(QStringLiteral("graphOperationOnHorizontalLayout"));
        graphAppOperateOnLabel = new QLabel(graphFileTab);
        graphAppOperateOnLabel->setObjectName(QStringLiteral("graphAppOperateOnLabel"));

        graphOperationOnHorizontalLayout->addWidget(graphAppOperateOnLabel);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Minimum, QSizePolicy::Minimum);

        graphOperationOnHorizontalLayout->addItem(horizontalSpacer_4);

        graphAppTypeComboBox = new QComboBox(graphFileTab);
        graphAppTypeComboBox->setObjectName(QStringLiteral("graphAppTypeComboBox"));
        sizePolicy.setHeightForWidth(graphAppTypeComboBox->sizePolicy().hasHeightForWidth());
        graphAppTypeComboBox->setSizePolicy(sizePolicy);

        graphOperationOnHorizontalLayout->addWidget(graphAppTypeComboBox);


        graphFileVerticalLayout->addLayout(graphOperationOnHorizontalLayout);

        graphFileOperationsHorizontalLayout = new QHBoxLayout();
        graphFileOperationsHorizontalLayout->setObjectName(QStringLiteral("graphFileOperationsHorizontalLayout"));
        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        graphFileOperationsHorizontalLayout->addItem(horizontalSpacer_6);

        loadGraphPushButton = new QPushButton(graphFileTab);
        loadGraphPushButton->setObjectName(QStringLiteral("loadGraphPushButton"));

        graphFileOperationsHorizontalLayout->addWidget(loadGraphPushButton);

        loadSaveHorizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        graphFileOperationsHorizontalLayout->addItem(loadSaveHorizontalSpacer);

        saveGraphPushButton = new QPushButton(graphFileTab);
        saveGraphPushButton->setObjectName(QStringLiteral("saveGraphPushButton"));

        graphFileOperationsHorizontalLayout->addWidget(saveGraphPushButton);

        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        graphFileOperationsHorizontalLayout->addItem(horizontalSpacer_7);


        graphFileVerticalLayout->addLayout(graphFileOperationsHorizontalLayout);


        gridLayout_3->addLayout(graphFileVerticalLayout, 0, 0, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_3->addItem(verticalSpacer, 1, 0, 1, 1);

        mainTabWidget->addTab(graphFileTab, QString());

        mainVerticalLayout->addWidget(mainTabWidget);


        gridLayout->addLayout(mainVerticalLayout, 0, 0, 1, 1);

        line_4 = new QFrame(graphCreatorPanelForm);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line_4, 1, 0, 1, 1);

        graphCreatorInfoTextEdit = new QTextEdit(graphCreatorPanelForm);
        graphCreatorInfoTextEdit->setObjectName(QStringLiteral("graphCreatorInfoTextEdit"));
        graphCreatorInfoTextEdit->setMinimumSize(QSize(0, 20));
        graphCreatorInfoTextEdit->setMaximumSize(QSize(16777215, 200));
        graphCreatorInfoTextEdit->setTextInteractionFlags(Qt::TextSelectableByKeyboard|Qt::TextSelectableByMouse);

        gridLayout->addWidget(graphCreatorInfoTextEdit, 2, 0, 1, 1);

        line_5 = new QFrame(graphCreatorPanelForm);
        line_5->setObjectName(QStringLiteral("line_5"));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line_5, 3, 0, 1, 1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_8);

        refreshButton = new QPushButton(graphCreatorPanelForm);
        refreshButton->setObjectName(QStringLiteral("refreshButton"));

        horizontalLayout_2->addWidget(refreshButton);

        horizontalSpacer_12 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_12);


        gridLayout->addLayout(horizontalLayout_2, 4, 0, 1, 1);


        retranslateUi(graphCreatorPanelForm);

        mainTabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(graphCreatorPanelForm);
    } // setupUi

    void retranslateUi(QWidget *graphCreatorPanelForm)
    {
        graphCreatorPanelForm->setWindowTitle(QApplication::translate("graphCreatorPanelForm", "Form", Q_NULLPTR));
        createVerticesPushButton->setText(QApplication::translate("graphCreatorPanelForm", "Create Vertices", Q_NULLPTR));
        createEdgesPushButton->setText(QApplication::translate("graphCreatorPanelForm", "Create Edges", Q_NULLPTR));
        edgeCreationTypelabel->setText(QApplication::translate("graphCreatorPanelForm", "Edge Creation Type:", Q_NULLPTR));
        edgeCreationTypeComboBox->clear();
        edgeCreationTypeComboBox->insertItems(0, QStringList()
         << QApplication::translate("graphCreatorPanelForm", "Straight Line", Q_NULLPTR)
         << QApplication::translate("graphCreatorPanelForm", "Cubic Bezier", Q_NULLPTR)
        );
#ifndef QT_NO_TOOLTIP
        edgeCreationTypeComboBox->setToolTip(QApplication::translate("graphCreatorPanelForm", "Edge type that will be created between vertices", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        edgeCreationTypeComboBox->setStatusTip(QString());
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_WHATSTHIS
        edgeCreationTypeComboBox->setWhatsThis(QString());
#endif // QT_NO_WHATSTHIS
        edgeCreationTypeComboBox->setCurrentText(QApplication::translate("graphCreatorPanelForm", "Straight Line", Q_NULLPTR));
        edgeDirectionTypeLabel->setText(QApplication::translate("graphCreatorPanelForm", "Edge Direction Type:", Q_NULLPTR));
        edgeDirectionTypeComboBox->clear();
        edgeDirectionTypeComboBox->insertItems(0, QStringList()
         << QApplication::translate("graphCreatorPanelForm", "Forward Only", Q_NULLPTR)
         << QApplication::translate("graphCreatorPanelForm", "Reverse Only", Q_NULLPTR)
         << QApplication::translate("graphCreatorPanelForm", "Bidirectional", Q_NULLPTR)
        );
        lockVerticesCheckBox->setText(QApplication::translate("graphCreatorPanelForm", "Lock All Vertices", Q_NULLPTR));
        lockEdgesCheckBox->setText(QApplication::translate("graphCreatorPanelForm", "Lock All Edges", Q_NULLPTR));
#ifndef QT_NO_WHATSTHIS
        graphElementTypeLabel->setWhatsThis(QApplication::translate("graphCreatorPanelForm", "Graph element type", Q_NULLPTR));
#endif // QT_NO_WHATSTHIS
        graphElementTypeLabel->setText(QString());
        mainTabWidget->setTabText(mainTabWidget->indexOf(graphModelingTab), QApplication::translate("graphCreatorPanelForm", "Graph Modeling", Q_NULLPTR));
        mainTabWidget->setTabToolTip(mainTabWidget->indexOf(graphModelingTab), QApplication::translate("graphCreatorPanelForm", "For creating a graph model", Q_NULLPTR));
        graphNameLabel->setText(QApplication::translate("graphCreatorPanelForm", "Graph Name:", Q_NULLPTR));
        graphFileFormatLabel->setText(QApplication::translate("graphCreatorPanelForm", "Graph File Format:", Q_NULLPTR));
        graphFileFormatComboBox->clear();
        graphFileFormatComboBox->insertItems(0, QStringList()
         << QApplication::translate("graphCreatorPanelForm", "GML", Q_NULLPTR)
         << QApplication::translate("graphCreatorPanelForm", "GraphML", Q_NULLPTR)
        );
        packageNamelabel->setText(QApplication::translate("graphCreatorPanelForm", "Package Name: ", Q_NULLPTR));
        graphFolderLabel->setText(QApplication::translate("graphCreatorPanelForm", "Graph Folder:", Q_NULLPTR));
        graphAppOperateOnLabel->setText(QApplication::translate("graphCreatorPanelForm", "Operate on:", Q_NULLPTR));
        graphAppTypeComboBox->clear();
        graphAppTypeComboBox->insertItems(0, QStringList()
         << QApplication::translate("graphCreatorPanelForm", "Graph Creator", Q_NULLPTR)
         << QApplication::translate("graphCreatorPanelForm", "Graph Server", Q_NULLPTR)
        );
        loadGraphPushButton->setText(QApplication::translate("graphCreatorPanelForm", "Load ", Q_NULLPTR));
        saveGraphPushButton->setText(QApplication::translate("graphCreatorPanelForm", "Save", Q_NULLPTR));
        mainTabWidget->setTabText(mainTabWidget->indexOf(graphFileTab), QApplication::translate("graphCreatorPanelForm", "Graph File", Q_NULLPTR));
        mainTabWidget->setTabToolTip(mainTabWidget->indexOf(graphFileTab), QApplication::translate("graphCreatorPanelForm", "For saving and loading graph files", Q_NULLPTR));
        graphCreatorInfoTextEdit->setPlaceholderText(QApplication::translate("graphCreatorPanelForm", "Graph creator noteworthy information will be displayed here", Q_NULLPTR));
        refreshButton->setText(QApplication::translate("graphCreatorPanelForm", "Refresh", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class graphCreatorPanelForm: public Ui_graphCreatorPanelForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // GRAPH_CREATOR_PANEL_UI_H
