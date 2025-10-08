#include "graph_creator_panel.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(graph_creator_rviz_plugin::GraphCreatorPanel,rviz::Panel )

namespace graph_creator_rviz_plugin
{

GraphCreatorPanel::GraphCreatorPanel( QWidget* parent ): 
  rviz::Panel( parent ), m_ui_ptr(new Ui::graphCreatorPanelForm())
{
  //Setting up the UI
  m_ui_ptr->setupUi(this);

  //Additional configuration for UI
  m_ui_ptr->createVerticesPushButton->setCheckable(true);
  m_ui_ptr->createVerticesPushButton->setChecked(true);
  m_ui_ptr->createEdgesPushButton->setCheckable(true);
  m_ui_ptr->createEdgesPushButton->setChecked(false);

  //UI signals
  //For graph modeling
  connect(m_ui_ptr->createVerticesPushButton, &QPushButton::clicked, this, &GraphCreatorPanel::onCreateVerticesClicked);
  connect(m_ui_ptr->createEdgesPushButton, &QPushButton::clicked, this, &GraphCreatorPanel::onCreateEdgesClicked);
  connect(m_ui_ptr->edgeCreationTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &GraphCreatorPanel::onEdgeCreationTypeIndexChanged);
  connect(m_ui_ptr->edgeDirectionTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &GraphCreatorPanel::onEdgeDirectionTypeIndexChanged);
  connect(m_ui_ptr->lockVerticesCheckBox, &QCheckBox::clicked, this, &GraphCreatorPanel::onLockAllVerticesClicked);
  connect(m_ui_ptr->lockEdgesCheckBox, &QCheckBox::clicked, this, &GraphCreatorPanel::onLockAllEdgesClicked);

  connect(m_ui_ptr->graphElementTableWidget, &QTableWidget::itemChanged, this, &GraphCreatorPanel::ongraphElementTableValueChanged);

  //For graph file
  connect(m_ui_ptr->graphAppTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &GraphCreatorPanel::onOperateOnAppTypeIndexChanged);
  connect(m_ui_ptr->loadGraphPushButton, &QPushButton::clicked, this, &GraphCreatorPanel::onLoadGraphClicked);
  connect(m_ui_ptr->saveGraphPushButton, &QPushButton::clicked, this, &GraphCreatorPanel::onSaveGraphClicked);

  connect(m_ui_ptr->refreshButton, &QPushButton::clicked, this, &GraphCreatorPanel::onRefreshClicked);

  //Publisher
  m_updated_vertex_pub = m_nh.advertise<graph_msgs::Vertex>("graph_creator/updates/vertex", 1, true);
  m_updated_edge_pub = m_nh.advertise<graph_msgs::Edge>("graph_creator/updates/edge", 1, true);

  //Subscribers
  m_current_vertex_sub = m_nh.subscribe("graph_creator/current/vertex", 1, &GraphCreatorPanel::currentVertexCallback, this);
  m_current_edge_sub = m_nh.subscribe("graph_creator/current/edge", 1, &GraphCreatorPanel::currentEdgeCallback, this);
  m_gc_state_sub = m_nh.subscribe("graph_creator/current_state", 1, &GraphCreatorPanel::currentGraphCreatorStateCallback, this);

  //Service clients
  m_set_gc_mode_clt = m_nh.serviceClient<graph_creator_msgs::SetGraphCreatorMode>("graph_creator/set_mode");
  m_set_et_clt = m_nh.serviceClient<graph_creator_msgs::SetEdgeCreationType>("graph_creator/set_edge_creation_type");
  m_set_ed_clt = m_nh.serviceClient<graph_creator_msgs::SetEdgeDirectionCreationType>("graph_creator/set_edge_direction_type");
  m_set_vert_lck_clt = m_nh.serviceClient<std_srvs::SetBool>("graph_creator/set_vertices_locked");
  m_set_edge_lck_clt = m_nh.serviceClient<std_srvs::SetBool>("graph_creator/set_edges_locked");
  m_get_gc_state_clt = m_nh.serviceClient<graph_creator_msgs::GetGraphCreatorState>("graph_creator/get_state");
  m_load_graph_creator_clt = m_nh.serviceClient<graph_msgs::LoadGraph>("graph_creator/load_graph");
  m_save_graph_creator_clt = m_nh.serviceClient<graph_msgs::SaveGraph>("graph_creator/save_graph");
  m_load_graph_server_clt = m_nh.serviceClient<graph_msgs::LoadGraph>("load_graph"); //For graph server
}

GraphCreatorPanel::~GraphCreatorPanel()
{
  delete m_ui_ptr;
}

void GraphCreatorPanel::currentVertexCallback(const graph_msgs::Vertex &vertex_msg)
{
  m_cur_graph_elem_type = GRAPH_ELEMENT_VERTEX;
  createVertexTable(vertex_msg);
}

void GraphCreatorPanel::currentEdgeCallback(const graph_msgs::Edge &edge_msg)
{
  m_cur_graph_elem_type = GRAPH_ELEMENT_EDGE;
  createEdgeTable(edge_msg);
}

void GraphCreatorPanel::createVertexTable(const graph_msgs::Vertex &vertex_msg)
{
  //TODO: This needs to be broken down into smaller functions
  ROS_INFO_STREAM("Received new vertex with name: "<<vertex_msg.name);

  m_cur_vertex_msg = vertex_msg;

  m_ui_ptr->graphElementTableWidget->clear();

  m_ui_ptr->graphElementTypeLabel->setText("Vertex");
  m_ui_ptr->graphElementTableWidget->blockSignals(true);

  m_ui_ptr->graphElementTableWidget->setRowCount(5);
  m_ui_ptr->graphElementTableWidget->setColumnCount(2);
  m_ui_ptr->graphElementTableWidget->setHorizontalHeaderLabels({"Field", "Value"});

  //ID
  int current_row = 0;

  resetItemDelegate(current_row);

  QTableWidgetItem *item = new QTableWidgetItem("ID");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  item = new QTableWidgetItem(QString::number(vertex_msg.id));
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);

  //Name
  current_row++;

  resetItemDelegate(current_row);

  item = new QTableWidgetItem("Name");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  item = new QTableWidgetItem(QString::fromStdString(vertex_msg.name));
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);

  //Alias
  current_row++;

  resetItemDelegate(current_row);

  item = new QTableWidgetItem("Alias");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  item = new QTableWidgetItem(QString::fromStdString(vertex_msg.alias));
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);

  //Type
  current_row++;

  resetItemDelegate(current_row);
  m_delegate_rows.insert(current_row);

  item = new QTableWidgetItem("Type");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  ComboBoxItemDelegate *vertex_type_cbid = new ComboBoxItemDelegate(m_ui_ptr->graphElementTableWidget);
  
  vertex_type_cbid->addItemsToComboBox(m_vertex_types);
  vertex_type_cbid->setCurrentItemInComboBox(vertex_msg.type);

  item = new QTableWidgetItem(QString::fromStdString(m_vertex_types.at(vertex_msg.type)));
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);
  m_ui_ptr->graphElementTableWidget->setItemDelegateForRow(current_row, vertex_type_cbid);

  //Properties
  current_row++;

  resetItemDelegate(current_row);
  m_delegate_rows.insert(current_row);

  item = new QTableWidgetItem("Properties");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  PropertiesDialogItemDelegate *vertex_pdid = new PropertiesDialogItemDelegate(m_ui_ptr->graphElementTableWidget);

  for(const graph_msgs::Property &property: vertex_msg.properties)
  {
    vertex_pdid->addKeyValueProperty(property.key, property.value);
  }

  item = new QTableWidgetItem();
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);
  m_ui_ptr->graphElementTableWidget->setItemDelegateForRow(current_row, vertex_pdid);

  m_ui_ptr->graphElementTableWidget->blockSignals(false);
}

void GraphCreatorPanel::createEdgeTable(const graph_msgs::Edge &edge_msg)
{
  //TODO: This can be broken down into simpler functions
  ROS_INFO_STREAM("Received new edge with name: "<<edge_msg.name);

  m_cur_edge_msg = edge_msg;

  m_ui_ptr->graphElementTypeLabel->setText("Edge");
  m_ui_ptr->graphElementTableWidget->blockSignals(true);

  m_ui_ptr->graphElementTableWidget->setRowCount(7);
  m_ui_ptr->graphElementTableWidget->setColumnCount(2);
  m_ui_ptr->graphElementTableWidget->setHorizontalHeaderLabels({"Field", "Value"});

  //Name
  int current_row = 0;

  resetItemDelegate(current_row);

  QTableWidgetItem *item = new QTableWidgetItem("Name");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  item = new QTableWidgetItem(QString::fromStdString(edge_msg.name));
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);

  //Alias
  current_row++;

  resetItemDelegate(current_row);

  item = new QTableWidgetItem("Alias");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  item = new QTableWidgetItem(QString::fromStdString(edge_msg.alias));
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);

  //Type
  current_row++;

  resetItemDelegate(current_row);
  m_delegate_rows.insert(current_row);

  item = new QTableWidgetItem("Type");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  ComboBoxItemDelegate* edge_type_cbid = new ComboBoxItemDelegate(m_ui_ptr->graphElementTableWidget);

  edge_type_cbid->addItemsToComboBox(m_edge_type);
  edge_type_cbid->setCurrentItemInComboBox(edge_msg.type);

  item = new QTableWidgetItem(QString::fromStdString(m_edge_type.at(edge_msg.type)));
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);
  m_ui_ptr->graphElementTableWidget->setItemDelegateForRow(current_row, edge_type_cbid);

  //Direction type
  current_row++;

  resetItemDelegate(current_row);
  m_delegate_rows.insert(current_row);

  item = new QTableWidgetItem("Direction Type");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  ComboBoxItemDelegate* edge_dir_type_cbid = new ComboBoxItemDelegate(m_ui_ptr->graphElementTableWidget);
  
  edge_dir_type_cbid->addItemsToComboBox(m_edge_direction_type);

  //FIXME: Currently the edge direction type depends on the bidirectionality boolean, this needs to moved to only use edge direction type field
  uint8_t edge_dir_type = graph_msgs::Edge::FORWARD;
  if(edge_msg.bidirectional) edge_dir_type = graph_msgs::Edge::BIDIRECTIONAL;

  edge_dir_type_cbid->setCurrentItemInComboBox(edge_dir_type);

  item = new QTableWidgetItem(QString::fromStdString(m_edge_direction_type.at(edge_dir_type)));
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);
  m_ui_ptr->graphElementTableWidget->setItemDelegateForRow(current_row, edge_dir_type_cbid);

  //Length
  current_row++;

  resetItemDelegate(current_row);

  item = new QTableWidgetItem("Length");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  item = new QTableWidgetItem(QString::number(edge_msg.length));
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);

  //Cost factor
  current_row++;

  resetItemDelegate(current_row);

  item = new QTableWidgetItem("Cost Factor");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  DoubleSpinBoxDelegate* cost_factor_dsbd = new DoubleSpinBoxDelegate(m_ui_ptr->graphElementTableWidget);
  cost_factor_dsbd->setDoubleSpinBoxProperties(0.0, 1000.0, 4, 0.1);
  cost_factor_dsbd->setCurrentValue(edge_msg.cost_factor);

  item = new QTableWidgetItem(QString::number(edge_msg.cost_factor));
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);
  m_ui_ptr->graphElementTableWidget->setItemDelegateForRow(current_row, cost_factor_dsbd);

  //Properties
  current_row++;

  resetItemDelegate(current_row);
  m_delegate_rows.insert(current_row);
  
  item = new QTableWidgetItem("Properties");
  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 0, item);

  PropertiesDialogItemDelegate *edge_pdid = new PropertiesDialogItemDelegate(m_ui_ptr->graphElementTableWidget);

  for(const graph_msgs::Property &property: edge_msg.properties)
  {
    edge_pdid->addKeyValueProperty(property.key, property.value);
  }
  
  item = new QTableWidgetItem();
  m_ui_ptr->graphElementTableWidget->setItem(current_row, 1, item);
  m_ui_ptr->graphElementTableWidget->setItemDelegateForRow(current_row, edge_pdid);

  m_ui_ptr->graphElementTableWidget->blockSignals(false);
}

void GraphCreatorPanel::resetItemDelegate(int current_row)
{
  if(m_delegate_rows.find(current_row) == m_delegate_rows.end()) return;

  QAbstractItemDelegate* item_delgate = m_ui_ptr->graphElementTableWidget->itemDelegateForRow(current_row);
  m_ui_ptr->graphElementTableWidget->setItemDelegateForRow(current_row, new QItemDelegate());
  delete item_delgate;
}

void GraphCreatorPanel::ongraphElementTableValueChanged(QTableWidgetItem *item)
{
  switch (m_cur_graph_elem_type)
  {
  case GRAPH_ELEMENT_VERTEX: updateTableVertexValueAndPublish(item); break;

  case GRAPH_ELEMENT_EDGE: updateTableEdgeValueAndPublish(item); break;
  
  default: 
  {
    ROS_ERROR_STREAM("Unknown graph element type was given");
    m_cur_graph_elem_type = GRAPH_ELEMENT_UNKNOWN;
    break;
  }
  }
}

void GraphCreatorPanel::updateTableVertexValueAndPublish(QTableWidgetItem *item)
{
  ROS_INFO_STREAM("Current vertex value changed");
  ROS_INFO_STREAM("Field row and column: ["<<item->row()<<" "<<item->column()<<"]");

  QTableWidgetItem* field_item = m_ui_ptr->graphElementTableWidget->item(item->row(), 0);
  ROS_INFO_STREAM("Field changed: "<<field_item->text().toStdString());
  ROS_INFO_STREAM("Field new value: "<<item->text().toStdString());

  graph_msgs::Vertex vertex_msg; //Updated vertex message
  vertex_msg = m_cur_vertex_msg;
  
  //Alias
  vertex_msg.alias = m_ui_ptr->graphElementTableWidget->item(2, 1)->text().toStdString();
  
  //Type
  //TODO: Can be improved
  std::string vertex_type_str = m_ui_ptr->graphElementTableWidget->item(3, 1)->text().toStdString();
  std::vector<std::string>::iterator vts_it = std::find(m_vertex_types.begin(), m_vertex_types.end(), vertex_type_str);
  vertex_msg.type = std::distance(m_vertex_types.begin(), vts_it);

  //Properties
  QMap<QString, QVariant> key_value_qmap = m_ui_ptr->graphElementTableWidget->item(4, 1)->data(Qt::DisplayRole).toMap();

  vertex_msg.properties.clear();

  for(auto kv_it = key_value_qmap.begin(); kv_it != key_value_qmap.end(); ++kv_it)
  {
    graph_msgs::Property property;
    property.key = kv_it.key().toStdString();
    property.value = kv_it.value().toString().toStdString();
    vertex_msg.properties.push_back(property);
  }

  m_updated_vertex_pub.publish(vertex_msg);
}

void GraphCreatorPanel::updateTableEdgeValueAndPublish(QTableWidgetItem *item)
{
  ROS_INFO_STREAM("Current edge value changed");
  ROS_INFO_STREAM("Field row and column: ["<<item->row()<<" "<<item->column()<<"]");

  QTableWidgetItem* field_item = m_ui_ptr->graphElementTableWidget->item(item->row(), 0);
  ROS_INFO_STREAM("Field changed: "<<field_item->text().toStdString());
  ROS_INFO_STREAM("Field new value: "<<item->text().toStdString());

  graph_msgs::Edge edge_msg;
  edge_msg = m_cur_edge_msg;

  //Alias
  edge_msg.alias = m_ui_ptr->graphElementTableWidget->item(1, 1)->text().toStdString();

  //Type
  std::string edge_type_str = m_ui_ptr->graphElementTableWidget->item(2, 1)->text().toStdString();
  std::vector<std::string>::iterator ets_it = std::find(m_edge_type.begin(), m_edge_type.end(), edge_type_str);
  edge_msg.type = std::distance(m_edge_type.begin(), ets_it);

  //Direction type
  std::string edge_dir_type_str = m_ui_ptr->graphElementTableWidget->item(3, 1)->text().toStdString();
  std::vector<std::string>::iterator vts_it = std::find(m_edge_direction_type.begin(), m_edge_direction_type.end(), edge_dir_type_str);
  edge_msg.edge_direction_type = std::distance(m_edge_direction_type.begin(), vts_it);
  
  //TODO: The bidirectional field might not be required after transistioning to the the edge direction type field
  if(edge_msg.edge_direction_type == graph_msgs::Edge::BIDIRECTIONAL)
    edge_msg.bidirectional = true;
  else
    edge_msg.bidirectional = false;

  //Cost factor
  edge_msg.cost_factor = m_ui_ptr->graphElementTableWidget->item(5, 1)->data(Qt::EditRole).toDouble();

  //Properties
  QMap<QString, QVariant> key_value_qmap = m_ui_ptr->graphElementTableWidget->item(6, 1)->data(Qt::DisplayRole).toMap();

  edge_msg.properties.clear();

  for(auto kv_it = key_value_qmap.begin(); kv_it != key_value_qmap.end(); ++kv_it)
  {
    graph_msgs::Property property;
    property.key = kv_it.key().toStdString();
    property.value = kv_it.value().toString().toStdString();
    edge_msg.properties.push_back(property);
  }

  m_updated_edge_pub.publish(edge_msg);
}

void GraphCreatorPanel::currentGraphCreatorStateCallback(const graph_creator_msgs::GraphCreatorState &gc_state_msg)
{
  setGraphCreatorState(gc_state_msg);
}

void GraphCreatorPanel::onCreateVerticesClicked()
{
  ROS_INFO_STREAM("Setting to create vertices mode");

  m_ui_ptr->createVerticesPushButton->setChecked(true);
  m_ui_ptr->createEdgesPushButton->setChecked(false);

  graph_creator_msgs::SetGraphCreatorMode set_mode_srv;
  set_mode_srv.request.mode.mode = graph_creator_msgs::GraphCreatorMode::GRAPH_CREATOR_MODE_CREATE_VERTICES;

  if(!m_set_gc_mode_clt.call(set_mode_srv))
  {
    ROS_ERROR_STREAM("Set graph creator mode to create vertices service call failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Service call to set graph creator mode to create vertices failed");
    return;
  }
}

void GraphCreatorPanel::onCreateEdgesClicked()
{
  ROS_INFO_STREAM("Setting to create edges mode");

  m_ui_ptr->createVerticesPushButton->setChecked(false);
  m_ui_ptr->createEdgesPushButton->setChecked(true);

  graph_creator_msgs::SetGraphCreatorMode set_mode_srv;
  set_mode_srv.request.mode.mode = graph_creator_msgs::GraphCreatorMode::GRAPH_CREATOR_MODE_CREATE_EDGES;

  if(!m_set_gc_mode_clt.call(set_mode_srv))
  {
    ROS_ERROR_STREAM("Set graph creator mode to create edges service call failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Service call to set graph creator mode to create edges failed");
    m_ui_ptr->createEdgesPushButton->setChecked(true);
    return;
  }
}

void GraphCreatorPanel::onEdgeCreationTypeIndexChanged(int index)
{
  ROS_INFO_STREAM("Edge creation type changed");

  graph_creator_msgs::SetEdgeCreationType set_et_srv;
  set_et_srv.request.type.type = index;

  if(!m_set_et_clt.call(set_et_srv))
  {
    ROS_ERROR_STREAM("Set edge creation type service call failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Setting edge creation type service call failed");
    m_ui_ptr->edgeCreationTypeComboBox->setCurrentIndex(m_cur_gc_state_msg.edge_creation_type.type); //Setting to previous index
    return;
  }
}
  
void GraphCreatorPanel::onEdgeDirectionTypeIndexChanged(int index)
{
  ROS_INFO_STREAM("Edge direction type changed");

  graph_creator_msgs::SetEdgeDirectionCreationType set_ed_srv;
  set_ed_srv.request.type.direction_type = index;

  if(!m_set_ed_clt.call(set_ed_srv))
  {
    ROS_ERROR_STREAM("Set edge direction type service call failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Setting edge direction type service call failed");
    m_ui_ptr->edgeDirectionTypeComboBox->setCurrentIndex(m_cur_gc_state_msg.edge_direction_creation_type.direction_type); //Setting to previous index
    return;
  }
}

void GraphCreatorPanel::onLockAllVerticesClicked()
{
  ROS_INFO_STREAM("Lock all vertices enabled: "<<std::boolalpha<<m_ui_ptr->lockVerticesCheckBox->isChecked());

  std_srvs::SetBool set_vert_lock_srv;
  set_vert_lock_srv.request.data = m_ui_ptr->lockVerticesCheckBox->isChecked();

  if(!m_set_vert_lck_clt.call(set_vert_lock_srv))
  {
    ROS_ERROR_STREAM("Set vertices lock service call failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Setting vertices lock service call failed");
    m_ui_ptr->lockVerticesCheckBox->setChecked(m_cur_gc_state_msg.all_vertices_locked);
    return;
  }
}

void GraphCreatorPanel::onLockAllEdgesClicked()
{
  ROS_INFO_STREAM("Lock all edges enable: "<<std::boolalpha<<m_ui_ptr->lockEdgesCheckBox->isChecked());

  std_srvs::SetBool set_edge_lock_srv;
  set_edge_lock_srv.request.data = m_ui_ptr->lockEdgesCheckBox->isChecked();

  if(!m_set_edge_lck_clt.call(set_edge_lock_srv))
  {
    ROS_ERROR_STREAM("Set edge lock service call failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Setting edges lock service call failed");
    m_ui_ptr->lockEdgesCheckBox->setChecked(m_cur_gc_state_msg.all_edges_locked);
    return;
  }
}

void GraphCreatorPanel::onRefreshClicked()
{
  ROS_INFO_STREAM("Refresh button clicked");

  graph_creator_msgs::GetGraphCreatorState get_gc_state_srv;

  if(!m_get_gc_state_clt.call(get_gc_state_srv))
  {
    ROS_ERROR_STREAM("Refesh of graph creator panel failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Could not refresh graph creator panel");
    return;
  }

  setGraphCreatorState(get_gc_state_srv.response.state);
}

void GraphCreatorPanel::onOperateOnAppTypeIndexChanged(int index)
{
  std::string selected_graph_app_type_str = m_ui_ptr->graphAppTypeComboBox->currentText().toStdString();
  
  if(selected_graph_app_type_str == "Graph Creator" && index == GRAPH_CREATOR)
  {
    m_cur_graph_app_type = index;
    m_cur_graph_app_type_str = "graph creator";
    ROS_INFO_STREAM("Operation type set to graph creator");
    
    //The order of the below steps matter
    m_ui_ptr->loadSaveHorizontalSpacer->changeSize(40, 20, QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_ui_ptr->saveGraphPushButton->setVisible(true);
  }
  else if(selected_graph_app_type_str == "Graph Server" && index == GRAPH_SERVER)
  {
    m_cur_graph_app_type = index;
    m_cur_graph_app_type_str = "graph server";
    ROS_INFO_STREAM("Operation type set to graph server");
    
    //The reverese order of the below steps needs to be done in order to undo these operations
    m_ui_ptr->saveGraphPushButton->setVisible(false);
    m_ui_ptr->loadSaveHorizontalSpacer->changeSize(0, 0, QSizePolicy::Fixed, QSizePolicy::Fixed);
  }
  else
  {
    ROS_ERROR_STREAM("An unknown opertion type was requested");
  }
}

void GraphCreatorPanel::onLoadGraphClicked()
{
  ROS_INFO_STREAM("Load graph activated");

  if(!loadGraphConfirmMessage())
  {
    ROS_WARN_STREAM("A load graph operation was rejected");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Load graph operation was rejected");
    return;
  }

  getGraphFileLoadSaveInformation();

  bool can_load = checkGraphFileLoadSaveInformation();

  if(!can_load)
  {
    ROS_ERROR_STREAM("Loading graph file failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Graph file loading failed");
    return;
  }

  std::stringstream pp_ss;
  std::string graph_extension_str;
  
  if(!getGraphFileExtensionFromFormat(m_graph_format, graph_extension_str))
  {
    ROS_ERROR_STREAM("An unknown graph extension was provided during loading");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Graph extension provided is of unknown type. Load failed");
    return;
  }

  if(!m_load_save_graph_from_local)
  {
    //This method assumes that at the service server side the load graph method implements package path resolution using rospack 
    pp_ss<<m_package_name<<"/"<<m_graph_folder<<"/"<<m_graph_name<<graph_extension_str; //Package relative path
    ROS_INFO_STREAM("Final load URL from local or remote package: "<<pp_ss.str());
  }
  else
  {
    //This method only checks locally on the machine where RViz is running 
    std::string package_path = ros::package::getPath(m_package_name);
    pp_ss<<package_path<<"/"<<m_graph_folder<<"/"<<m_graph_name<<graph_extension_str; //Absolute path
    ROS_INFO_STREAM("Final load URL from local package: "<<pp_ss.str());
  }

  //TODO: A method is required to browse graph file locally, load it and set it in graph creator 

  graph_msgs::LoadGraph load_graph_srv;
  load_graph_srv.request.graph_url = pp_ss.str();
  
  if(m_cur_graph_app_type == GRAPH_CREATOR)
  {
    if(!m_load_graph_creator_clt.call(load_graph_srv))
    {
      ROS_ERROR_STREAM("Load graph service call to graph creator failed");
      m_ui_ptr->graphCreatorInfoTextEdit->append("Load graph service call to graph creator failed");
      return;
    }
  }
  else if(m_cur_graph_app_type == GRAPH_SERVER)
  {
    if(!m_load_graph_server_clt.call(load_graph_srv))
    {
      ROS_ERROR_STREAM("Load graph service call to graph server failed");
      m_ui_ptr->graphCreatorInfoTextEdit->append("Load graph service call to graph server failed");
      return;
    }
  }

  if(load_graph_srv.response.result == graph_msgs::LoadGraph::Response::RESULT_SUCCESS)
  {
    ROS_INFO_STREAM("Loaded graph with name "<<m_graph_name<<" to "<<m_cur_graph_app_type_str);
    m_ui_ptr->graphCreatorInfoTextEdit->append(QString::fromStdString("Loaded graph with name "+m_graph_name+" to "+m_cur_graph_app_type_str));
  }
  else
  {
    ROS_ERROR_STREAM("Loading graph with name "<<m_graph_name<<" to "<<m_cur_graph_app_type_str<<" failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append(QString::fromStdString("Loading graph with name "+m_graph_name+" to "+m_cur_graph_app_type_str+" failed"));
  }
}

void GraphCreatorPanel::onSaveGraphClicked()
{
  if(m_cur_graph_app_type == GRAPH_SERVER)
  {
    ROS_ERROR_STREAM("Save graph is not a valid operation while current graph application to operation on graph server");
    return;
  }

  ROS_INFO_STREAM("Save graph activated");

  if(!saveGraphConfirmMessage())
  {
    ROS_WARN_STREAM("A save graph operation was rejected");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Save graph operation was rejected");
    return;
  }

  getGraphFileLoadSaveInformation();

  bool can_save = checkGraphFileLoadSaveInformation();

  if(!can_save) 
  {
    ROS_ERROR_STREAM("Saving graph failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Graph save failed");
    return;
  }
  
  std::stringstream pp_ss;

  std::string graph_extension_str;
  
  if(!getGraphFileExtensionFromFormat(m_graph_format, graph_extension_str))
  {
    ROS_ERROR_STREAM("An unknown graph extension was provided during saving");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Graph extension provided is of unknown type. Save failed");
    return;
  }

  if(!m_load_save_graph_from_local)
  {
    pp_ss<<m_package_name<<"/"<<m_graph_folder<<"/";
    ROS_INFO_STREAM("Final save URL to local or remote package: "<<pp_ss.str()<<m_graph_name<<graph_extension_str);
  }
  else
  {
    std::string package_path = ros::package::getPath(m_package_name);
    pp_ss<<package_path<<"/"<<m_graph_folder<<"/";
    ROS_INFO_STREAM("Final save URL to local package: "<<pp_ss.str()<<m_graph_name<<graph_extension_str);
  }

  //TODO: A method is required to browse to a folder and save the graph file in the selected folder

  graph_msgs::SaveGraph save_graph_srv;
  save_graph_srv.request.graph_file_name = m_graph_name;
  save_graph_srv.request.save_graph_url = pp_ss.str();

  if(!m_save_graph_creator_clt.call(save_graph_srv))
  {
    ROS_ERROR_STREAM("Save graph service call to graph creator failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Save graph service call to graph creator failed");
    return;
  }

  QString save_graph_info_str = "";
  if(save_graph_srv.response.result == graph_msgs::SaveGraph::Response::RESULT_SUCCESS)
  {
    ROS_INFO_STREAM("Saved graph from"<<m_cur_graph_app_type_str<<" with name "<<m_graph_name<<graph_extension_str<<" to "<<pp_ss.str());
    save_graph_info_str = QString::fromStdString("Saved graph from "+m_cur_graph_app_type_str+" with name "+m_graph_name+graph_extension_str+" to "+pp_ss.str());
    m_ui_ptr->graphCreatorInfoTextEdit->append(save_graph_info_str);
  }
  else
  {
    ROS_ERROR_STREAM("Save graph from "<<m_cur_graph_app_type_str<<" with name "<<m_graph_name<<graph_extension_str<<" to "<<pp_ss.str()<<" failed");
    save_graph_info_str = QString::fromStdString("Saving graph from "+m_cur_graph_app_type_str+" with name "+m_graph_name+graph_extension_str+" to "+pp_ss.str()+" failed");
    m_ui_ptr->graphCreatorInfoTextEdit->append(save_graph_info_str);
  }
}

bool GraphCreatorPanel::loadGraphConfirmMessage()
{
  //Load a graph file to server is not a destructive operation and therefore this confirmation is not required
  if(m_cur_graph_app_type == GRAPH_SERVER) return true;

  QMessageBox::StandardButton reply;
  reply = QMessageBox::warning(this, "Load Graph", 
                              "Loading a new graph will erase any unsaved graph in graph creator.\n\nDo you want to continue?", 
                               QMessageBox::Yes | QMessageBox::No,
                               QMessageBox::No);

  if(reply == QMessageBox::Yes) return true;
  
  return false;
}
  
bool GraphCreatorPanel::saveGraphConfirmMessage()
{
  QMessageBox::StandardButton reply;
  reply = QMessageBox::warning(this, "Save Graph", 
                              "Saving current graph will overwrite any graph file with same name and extension.\n\nDo you want to continue?", 
                               QMessageBox::Yes | QMessageBox::No,
                               QMessageBox::No);

  if(reply == QMessageBox::Yes) return true;
  
  return false;
}

void GraphCreatorPanel::getGraphFileLoadSaveInformation()
{
  m_graph_name = m_ui_ptr->graphNameLineEdit->text().toStdString();
  m_package_name = m_ui_ptr->packageNameLineEdit->text().toStdString();
  m_graph_format = m_ui_ptr->graphFileFormatComboBox->currentText().toStdString();
  m_graph_folder = m_ui_ptr->graphFolderLineEdit->text().toStdString();

  ROS_INFO_STREAM("Graph name: "<<m_graph_name);
  ROS_INFO_STREAM("Graph format: "<<m_graph_format);
  ROS_INFO_STREAM("Package name: "<<m_package_name);
  ROS_INFO_STREAM("Graph Folder: "<<m_graph_folder);
}

bool GraphCreatorPanel::getGraphFileExtensionFromFormat(const std::string &format, std::string &extension)
{
  extension = "";

  if(m_graph_format == "GraphML") extension = ".graphml";
  else if(m_graph_format == "GML") extension = ".gml";
  else return false;

  return true;
}

bool GraphCreatorPanel::checkGraphFileLoadSaveInformation()
{
  bool can_load_save = true;

  if(m_graph_name.empty())
  {
    ROS_ERROR_STREAM("Graph name is empty");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Graph name is empty");
    can_load_save = false;
  }

  if(m_package_name.empty())
  {
    ROS_ERROR_STREAM("Package in which the graphs are save was not provided");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Package in which graphs are saved was not provided");
    can_load_save = false;
  }

  if(m_graph_folder.empty())
  {
    ROS_ERROR_STREAM("Graph folder name is empty");
    m_ui_ptr->graphCreatorInfoTextEdit->append("Graph folder was not provided");
    can_load_save = false;
  }

  return can_load_save;
}

void GraphCreatorPanel::setGraphCreatorState(const graph_creator_msgs::GraphCreatorState &gc_state_msg)
{
  m_cur_gc_state_msg = gc_state_msg; //Update to the latest state of graph creator

  if(gc_state_msg.graph_creator_mode.mode == graph_creator_msgs::GraphCreatorMode::GRAPH_CREATOR_MODE_CREATE_VERTICES)
  {
    m_ui_ptr->createVerticesPushButton->setChecked(true);
    m_ui_ptr->createEdgesPushButton->setChecked(false);
  }
  else if(gc_state_msg.graph_creator_mode.mode == graph_creator_msgs::GraphCreatorMode::GRAPH_CREATOR_MODE_CREATE_EDGES)
  {
    m_ui_ptr->createVerticesPushButton->setChecked(false);
    m_ui_ptr->createEdgesPushButton->setChecked(true);
  }
  
  m_ui_ptr->edgeCreationTypeComboBox->setCurrentIndex(gc_state_msg.edge_creation_type.type);
  m_ui_ptr->edgeDirectionTypeComboBox->setCurrentIndex(gc_state_msg.edge_direction_creation_type.direction_type);

  m_ui_ptr->lockVerticesCheckBox->setChecked(gc_state_msg.all_vertices_locked);
  m_ui_ptr->lockEdgesCheckBox->setChecked(gc_state_msg.all_edges_locked);
}

void GraphCreatorPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);

  config.mapSetValue("graph_name", QString::fromStdString(m_graph_name));
  config.mapSetValue("graph_format", QString::fromStdString(m_graph_format));
  config.mapSetValue("package_name", QString::fromStdString(m_package_name));
  config.mapSetValue("graph_folder", QString::fromStdString(m_graph_folder));
}

void GraphCreatorPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);

  QString out_string;
  if(config.mapGetString("graph_name", &out_string))
  {
    m_graph_name = out_string.toStdString();
    m_ui_ptr->graphNameLineEdit->setText(out_string);
  }

  out_string = "";
  if(config.mapGetString("graph_format", &out_string))
  {
    m_graph_format = out_string.toStdString();
    int req_index = m_ui_ptr->graphFileFormatComboBox->findText(out_string);
    m_ui_ptr->graphFileFormatComboBox->setCurrentIndex(req_index);
  }

  out_string = "";
  if(config.mapGetString("package_name", &out_string))
  {
    m_package_name = out_string.toStdString();
    m_ui_ptr->packageNameLineEdit->setText(out_string);
  }

  out_string = "";
  if(config.mapGetString("graph_folder", &out_string))
  {
    m_graph_folder = out_string.toStdString();
    m_ui_ptr->graphFolderLineEdit->setText(out_string);
  }
}

}
