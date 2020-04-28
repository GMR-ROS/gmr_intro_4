# Introdução de ROS e robôs móveis terrestres

Após incrementarmos RobotClass com o [publicador de odometria](https://bitbucket.org/grupomecatronica/gmr_intro_3), vamos explorar um pouco de serviços. 

## ROS Services

Os serviços ROS são uma alternativa ao modelo publicador-subscritor, em que a comunicação é contínua após ser estabelecida, mas é de via única: o publicador manda mensagem ao subscritor, mas o publicador em si não consegue saber nada do seu subscritor. Já os serviços são eventos pontuais, em que um cliente faz uma requisição ao servidor. Análogo ao mundo real, é possível que o servidor retorne uma resposta à requisição feita. 

Assim como os tópicos utilizam mensagens ROS para carregar suas informações, os serviços possuem suas formas estruturadas de informação. Por exemplo, informações sobre o serviço roscpp_tutorials/TwoInts podem ser obtidas com: 
```console
user@pc:~$ rossrv info roscpp_tutorials/TwoInts
int64 a
int64 b
---
int64 sum
```

Observe que a descrição de um serviço possui duas seções, delimitadas pelo marcador `---`. A primeira seção possui os dados referentes ao cliente. Já a segunda contém a resposta envia pelo servidor após executar o serviço. Neste caso, o cliente envia dois inteiros, a e b, e o servidor retorna a soma deles, sum. Agora observe o serviço std_srvs/Trigger:
```console
user@pc:~$ rossrv info std_srvs/Trigger
---
bool success
string message
```
Como o próprio nome indica, trata-se apenas de um trigger. O cliente só faz a solicitação do serviço sem enviar nenhum dado. Por outro lado, o servidor responde com um booleano indicando se o serviço foi bem-sucedido e com uma string. 

Da mesma forma que as mensagens ROS, podemos encontrar algumas que já vêm instaladas. Mas são poucas e muito mais difíceis de generalizar. Caso necessário, serviços personalizados podem ser criados no seu pacote. 

## Nosso caso
Ao rodar `roslaunch gmr_intro_poo robot.launch` ou simplesmente `rosrun gmr_intro gmr_intro_node` e, numa sessão adicional de terminal, executar:
```console
user@pc:~$  rosservice list
/gmr_intro_node/get_loggers
/gmr_intro_node/set_logger_level
/gmr_intro_poo_node/get_loggers
/gmr_intro_poo_node/set_logger_level
/rosout/get_loggers
/rosout/set_logger_level
/toggle_robot
```
Observamos que cada nó ativo possui ao menos dois serviços associados: get_loggers e set_logger_level. Eles estão relacionados às mensagens de saída ao console/[logging](http://wiki.ros.org/roscpp/Overview/Logging), ou seja, quando utilizamos ROS_INFO ou ROS_INFO_STREAM, por exemplo. Há outros níveis de verbosidade (na ordem, DEBUG, INFO, WARN, ERROR e FATAL) para serem utilizados. Podemos utilizar os serviços mencionados para verificar/configurar o nível de verbosidade que será impresso no terminal. 

Temos um outro serviço disponível: `/toggle_robot`. Para obter informações sobre ele:
```console
user@pc:~$ rosservice info /toggle_robot 
Node: /gmr_intro_node
URI: rosrpc://pc:42811
Type: std_srvs/Trigger
Args: 
```
Obtemos qual é o nó servidor, a combinação hostname:port (pc:42811) pela qual a comunicação será estabelecida, o tipo de serviço e os argumentos necessários para fazer uma requisição do serviço. Como esperado (vimos acima com rossrv info std_srvs/Trigger), trata-se de um serviço que não precisa de argumentos para ser solicitado. Podemos, por exemplo, fazer uma requisição do serviço /toggle_robot através da linha de comando:
```console
user@pc:~$ rosservice call /toggle_robot 
success: True
message: "Someone toggled me..."
```
Observe que recebemos a resposta do servidor e se olharmos a saída de nosso gmr_intro_poo_node, as velocidades estão zeradas. De fato, `rostopic echo /left_rpm` e `rostopic echo /right_rpm` confirmam isso. Se fizermos a requisição do serviço novamente, as velocidades deixam de ser nulas, ou seja, o robô ficticiamente volta a se movimentar.

## Requisição de serviços dentro de um nó ROS em C++

Veremos agora como fazer a requisição do serviço por código. Vamos incrementar o nosso RobotClass(https://bitbucket.org/grupomecatronica/gmr_intro_poo/tree/gmr_intro_3) para que ele chame /toggle_robot a cada n segundos, em que n é um parâmetro ROS. Primeiro, editaremos o cabeçalho da classe em gmr_intro_poo/include/gmr_intro_poo/gmr_intro_poo.hpp, adicionando a biblioteca relacionada ao serviço:

```cpp
#include <std_srvs/Trigger.h>
```
Adicionaremos o protótipo do método público (será chamado em gmr_intro_poo_node.cpp) que fará a requisição de serviço observando o intervalo de tempo:
```cpp
        void                checkToggleRobot();
```        
Adicionaremos um cliente de serviço como membro privado:
```cpp
        ros::ServiceClient  _client_toggle_robot;
```
Como desejamos que a requisição do serviço ocorra em tempos regulares e definidos por um parâmetro `time_between_toggles`, adicionemos um campo na struct Params:
```cpp

        struct Params
        {
            double          time_between_toggles;
            double          axle_track;
            double          gear_ratio;
            double          wheel_radius;
            std::string     topic_name_left_rpm;
            std::string     topic_name_right_rpm;
        }_params;
```
E também um membro privado para manter o tempo da última requisição:
```cpp
        ros::Time _prev_timestamp_toggle;
```
Agora editaremos a implementação da classe em gmr_intro_poo/src/gmr_intro_poo.cpp. No construtor da classe RobotClass::RobotClass(ros::NodeHandle* nh), precisamos inicializar o cliente, obter o parâmetro e atribuir o valor inicial ao _prev_timestamp_toggle.
```cpp

    _client_toggle_robot = _nh->serviceClient<std_srvs::Trigger>("/toggle_robot");

    _nh->param("time_between_toggles", _param.time_between_toggles, -1.0);

    _prev_timestamp_toggle = _prev_timestamp;
```
Deixaremos um valor padrão de -1.0 para _param.time_between_toggles de modo que criaremos a lógica de que a requisição será feita somente para tempos positivos. Assim, caso o parâmetro não seja iniciado, o cliente também não solicitará o serviço. 

Em seguida, vamos definir o método RobotClass::checkToggleRobot():

```cpp
void RobotClass::checkToggleRobot()
{
    ros::Time cur_timestamp = ros::Time::now();
    std_srvs::Trigger srv;
    if(_params.time_between_toggles > 0  && cur_timestamp.toSec() - _prev_timestamp_toggle.toSec() > _params.time_between_toggles)
    {
        _prev_timestamp_toggle = cur_timestamp;
        if(_client_toggle_robot.call(srv))
        {
            ROS_WARN_STREAM("message: " << srv.response.message);
        }
        else 
        {
           ROS_ERROR("Falha em chamar o serviço");
        }
    }
}
```
Observe que o condicional `if(_params.time_between_toggles > 0  && cur_timestamp.toSec() - _prev_timestamp_toggle.toSec() > _params.time_between_toggles)` requer duas condições: o parâmetro deverá ser maior que zero e o tempo passado em relação à última requisição deverá ser maior que o parâmetro especificado. Com isso, temos liberdade de não chamar o serviço (apenas atribuir um valor negativo ou zero ao parâmetro). 

A solicitação de serviço é feita com um método que retorna um booleano que informa se foi possível chamar o serviço. O método necessita de um único argumento que é srv, um std_srvs::Trigger. Lembre-se que a descrição de um serviço possui dois campos. Para atribuir valores na seção do requisitante, utilizamos srv.request.**campo**. Aqui não temos nenhum para preencher. E para obter a resposta enviada pelo servidor, utilizamos srv.response.**campo**. No lugar de ROS_INFO_STREAM, usaremos ROS_WARN_STREAM.

Precisamos incluir o método RobotClass::checkToggleRobot() no loop explícito em gmr_intro_poo/src/gmr_intro_poo_node.cpp:

```cpp
    while(ros::ok())
    {
        robot.checkToggleRobot();
        robot.calculateOdom();
        loop_rate.sleep();
        ros::spinOnce();
    }
```

Por fim, devemos adicionar o parâmetro time_between_toggles ao gmr_intro_poo/launch/robot.launch. Note que é um parâmetro local então deve ficar dentro do nó de nome gmr_intro_poo_node. Testaremos para um intervalo de 2 segundos:

```xml
  <node pkg="gmr_intro_poo" type="gmr_intro_poo_node" name="gmr_intro_poo_node" output="screen" required="true">
    <param name="nome_topico_left_rpm" value="/left_rpm"/>
    <param name="nome_topico_right_rpm" value="/right_rpm"/>
    <param name="time_between_toggles" value="2.0"/>
  </node>
```

Agora que editamos todos os arquivos, compile
```console
user@pc:~/ros_ws$ catkin_make
```
e rode o arquivo launch:
```console
user@pc:~$ roslaunch gmr_intro_poo robot.launch
```
Observe que teremos uma saída do tipo:
```
[ INFO] [1587470168.158720537]: velocidade linear esquerda: 0.0235954
[ INFO] [1587470168.158843463]: velocidade linear direita: 0.023911
[ INFO] [1587470168.178687286]: velocidade linear esquerda: 0.0190105
[ INFO] [1587470168.178765634]: velocidade linear direita: 0.0198796
[ INFO] [1587470168.198698409]: velocidade linear esquerda: 0.0230288
[ INFO] [1587470168.198783017]: velocidade linear direita: 0.0134457
[ WARN] [1587470168.214919099]: message: Someone toggled me...
[ INFO] [1587470168.218665485]: velocidade linear esquerda: 0.0128
[ INFO] [1587470168.218763591]: velocidade linear direita: 0.016336
[ INFO] [1587470168.238780676]: velocidade linear esquerda: 0
[ INFO] [1587470168.238902936]: velocidade linear direita: 0
[ INFO] [1587470168.258710836]: velocidade linear esquerda: 0
[ INFO] [1587470168.258843057]: velocidade linear direita: 0
[ INFO] [1587470168.278708086]: velocidade linear esquerda: 0
```
*Obs.:* A sétima linha (a que inicia com [WARN]) deverá aparecer em amarelo no seu terminal.

Vamos analisar essa saída de terminal. Note primeiro que temos dois níveis de verbosidade: [INFO] e [WARN]. Os vários níveis ajudam a separar os diferentes tipos de printouts, inclusive visualmente. O próximo campo é o timestamp, o tempo de sistema em segundos e com casas decimais até nanosegundos. Agora focaremos na saída de um dos subscritores (e.g. RobotClass::subLeft). Ela mostra a velocidade linear esquerda assim que recebe a mensagem ("/left_rpm). Pelos timestamps, podemos ver que acontece a cada 20 ms (*.158 -> *.178 - *.198 -> *.218). No timestamp *.214 aconteceu uma requisição do serviço. Não temos saídas de /gmr_intro_node para confirmar, mas podemos supor que as velocidades já haviam sido publicadas antes da solicitação do serviço e o comportamento esperado de zerar as velocidades ocorre no próximo loop. Caso queira analisar mais, adicione ROS_INFO/ROS_WARN/outros em gmr_intro/src/gmr_intro_node.

Abra outra sessão de terminal (preferencialmente um split vertical no terminator). Primeiro, veremos os loggers de /gmr_intro_poo:
```console
user@pc:~$ rosservice call /gmr_intro_poo_node/get_loggers 
loggers: 
  - 
    name: "ros"
    level: "info"
  - 
    name: "ros.gmr_intro_poo"
    level: "info"
  - 
    name: "ros.roscpp"
    level: "info"
  - 
    name: "ros.roscpp.roscpp_internal"
    level: "info"
  - 
    name: "ros.roscpp.roscpp_internal.connections"
    level: "info"
  - 
    name: "ros.roscpp.superdebug"
    level: "warn"
```
Observe, por exemplo, que o nível de verbosidade para `ros.gmr_intro_poo` é INFO. De acordo com o descrito em [roscpp/Overview/Logging](http://wiki.ros.org/roscpp/Overview/Logging), INFO é o segundo mais baixo. Como o nível está configurado a ele, todas as saídas acima dele, ele incluso, serão exibidas. Então vamos alterar para que seja nível WARN:
```console
user@pc:~$ rosservice call /gmr_intro_poo_node/set_logger_level ros.gmr_intro_poo warn
```
Observaremos na sessão com o launch:
```console
[ INFO] [1587472379.254119179]: velocidade linear esquerda: -0.0736256
[ INFO] [1587472379.254149292]: velocidade linear direita: 0.0825568
[ INFO] [1587472379.274184106]: velocidade linear esquerda: -0.0750989
[ INFO] [1587472379.274307638]: velocidade linear direita: 0.0775173
[ INFO] [1587472379.294158374]: velocidade linear esquerda: -0.0684985
[ INFO] [1587472379.294281203]: velocidade linear direita: 0.0871206
[ INFO] [1587472379.314158433]: velocidade linear esquerda: -0.0736139
[ INFO] [1587472379.334169086]: velocidade linear direita: 0.0713845
[ WARN] [1587472379.570454370]: message: Someone toggled me...
[ INFO] [1587472379.710276794]: Toggle
[ INFO] [1587472379.730248274]: Toggle
[ INFO] [1587472379.793709678]: Toggle
[ INFO] [1587472379.853734124]: Toggle
[ INFO] [1587472379.911935445]: Toggle
[ INFO] [1587472379.930895118]: Toggle
[ INFO] [1587472379.950858783]: Toggle
```
Após configurar o nível de verbosidade para WARN (terceiro em ordem), não há mais saídas do tipo INFO para o nó que configuramos, gmr_intro_poo_node. 

## Mas e como é o servidor?

Podemos ver um exemplo de servidor em gmr_intro/src/gmr_intro_node.cpp. Primeiro observe que POO não foi utilizado. Por causa disso, uma variável global bool toggle_robot foi necessária para ter uma informação através das funções. Apesar de ser mais fácil de criar um nó assim, não é uma boa prática. Conseguimos manter certo controle para códigos pequenos, mas tal controle perde-se facilmente à medida que a complexidade aumenta e podemos ter conflitos devido à variável global. Voltando à criação do servidor, precisamos: 1) Adicionar a biblioteca correspondente, inicializar o servidor e criar uma função correspondente.

Adição da biblioteca:
```cpp
#include <std_srvs/Trigger.h>
```

Inicialização do servidor:
```cpp
  ros::ServiceServer service = nh.advertiseService("/toggle_robot", toggleRobot);
```
Observe que definimos o nome do serviço no primeiro argumento e vinculamos a uma função no segundo argumento.

Criação da função correspondente:
```cpp
bool toggleRobot(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  toggle_robot = !toggle_robot;
  response.success = true;
  response.message = "Someone toggled me...";
  return true;
}
```
Observe que a função tem um formato padrão. Ela retorna booleano e possui dois argumentos passados por referência. Os dois argumentos são as seções da descrição do serviço, a requisição (std_srvs::Trigger::Request) e a resposta (std_srvs::Trigger::Response). De forma geral, executamos o serviço de acordo com os dados de request, alteramos response e a função retorna se conseguiu ou não efetuar o serviço.

De forma similar ao subscritor, o servidor só precisa ser inicializado, mas não precisa ser chamado posteriormente. Basta que haja um ros::spin ou um loop com ros::spinOnce que a função associada ao servidor será executada quando for solicitada por alguém.

Os códigos estão disponíveis em [gmr_intro_poo](https://bitbucket.org/grupomecatronica/gmr_intro_poo) no branch gmr_intro_4. Com isso, a nossa classe RobotClass adquiriu o poder de requisitar o serviço /toggle_robot. No próximo tutorial, veremos a ferramenta de visualização rviz.


*Para um outro exemplo de ROS services, inclusive abordando a criação de um serviço personalizado, veja o post de [Embarcados](https://www.embarcados.com.br/ros-services/).*


Contato: Akihiro (akihirohh@gmail.com)

Apoio da Fundação de Amparo à Pesquisa do Estado de São Paulo (FAPESP) através do processo nº 2018/10894-2. 
As opiniões, hipóteses e conclusões ou recomendações expressas neste material são de responsabilidade do(s) autor(es) e não necessariamente refletem a visão da FAPESP.