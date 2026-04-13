# Instalação e execução do blender

- O instalador ou portavel do blender pode ser obtido em: https://www.blender.org/download/, disponível para todos os sistemas operacionais. Segue o tutorial para ubuntu.
    
    ```sh
    wget -c https://download.blender.org/release/Blender5.1/blender-5.1.0-linux-x64.tar.xz
    tar -xf blender-5.1.0-linux-x64.tar.xz
    cd blender-5.1.0-linux-x64
    ./blender
    ```
    

# Instalação do Blosm

- O Blosm é um plugin para o blender que permite importar mapas 3D de provedores. Ele pode ser obtido gratuitamente (também existe uma versão paga) no seguinte link:
    - Gratuita: https://prochitecture.gumroad.com/l/blender-osm
    - Pro: https://prochitecture.gumroad.com/l/blosm
- É necessário prover um endereço de email para obter o plugin

Após obter o arquivo, deve-se realizar os seguintes passos:

- Abrir o blender e navegar até edit > preferences > add-ons > “⌄” na parte superior direita > Install from disk > selecionar o arquivo blosm .zip > Install from disk


<div align="center">
  <img src="images/blosm-pref.png" alt="" style="max-width: 80%; height: auto;" />
</div>


<div align="center">
  <img src="images/blosm-addons.png" alt="" style="max-width: 80%; height: auto;" />
</div>


<div align="center">
  <img src="images/blosm-select.png" alt="" style="max-width: 80%; height: auto;" />
</div>

# Obter API Google Tile

### Requisito
- Conta Google Cloud: https://docs.cloud.google.com/docs/get-started

### Passos para obter a chave API do Google Tile

Com uma conta google cloud é possível obter a chave API do Google Tile, que é necessária para o Blosm obter os dados do google e gerar mapas mais próximos do real. Para isso, basta seguir os seguintes passos:


- Acessar o console do [google cloud](https://console.cloud.google.com/) e pressionar no projeto atual para abrir a lista de projetos.

<div align="center">
  <img src="images/key-press-project.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Pressionar no botão “Novo Projeto” para criar um novo projeto.

<div align="center">
  <img src="images/key-create.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Nomeie o projeto e selecione a organização e conta de faturamento. 
  - Obs.: Apesar de ser utilizada uma conta de faturamento, os primeiros 1000 tiles do mês são gratuitos, logo, para gerar algumas dezenas de mapas pequenos não haverá custo.

<div align="center">
  <img src="images/key-set-name-and-confirm.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Após confirmar a criação do projeto, a pagina é redirecionada para o dashboard do projeto e uma notificação de criação do projeto é exibida, basta pressionar no botão "Selecionar Projeto" para acessar o projeto criado.

<div align="center">
  <img src="images/key-project-confirmation.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- No dashboard do projeto, basta abrir o menu lateral pressionando no ícone de três linhas no canto superior esquerdo da tela e navegar até Plataforma Google Maps > Chaves e credenciais.

<div align="center">
  <img src="images/key-open-menu.png" alt="" style="max-width: 80%; height: auto;" />
</div>

<div align="center">
  <img src="images/key-select-keys.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Uma nova aba é aberta com a chave api do projeto, desative a opção "Ativar todas as APIs do Google Maps neste projeto" e pressione "Acessar a Plataforma...".

<div align="center">
  <img src="images/key-key-created.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Na nova aba, pressione "Talver mais tarde".

<div align="center">
  <img src="images/key-skip.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- No dashboard da plataforma google maps, pressione no menu lateral "Chaves e credenciais".

<div align="center">
  <img src="images/key-sec-menu.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Em seguida, na Chave API existente, pressione nos três pontos e selecione "Editar chave de API".

<div align="center">
  <img src="images/key-edit.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Na nova aba, selecione apenas a opção "Map Tiles API" nas restrições de API e salve as alterações.

<div align="center">
  <img src="images/key-select-map-tile.png" alt="" style="max-width: 80%; height: auto;" />
</div>

<div align="center">
  <img src="images/key-confirm-edit.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Novamente no dashboard da plataforma google maps, pressione "Exibir chaves de API" para obter a chave API do Google Tile.

<div align="center">
  <img src="images/key-copy.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Abra o blender, navegue até edit > preferences > add-ons > Blosm e adicione a chave API do Google Tile no campo "Google 3D Tiles Key", pressione enter para salvar a chave e feche a janela de preferências.

<div align="center">
  <img src="images/key-blender-config.png" alt="" style="max-width: 80%; height: auto;" />
</div>

<div align="center">
  <img src="images/key-blender-set.png" alt="" style="max-width: 80%; height: auto;" />
</div>

# Gerar mapa

Com a chave API do Google Tile adicionada é possível gerar mapas mais próximos do real com textura através do blosm, seguem os passos:

- Expandir o menu lateral apertando no botão “<”
    
<div align="center">
  <img src="images/map-lateral-menu.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Selecionar a opção do Blosm
    
<div align="center">
  <img src="images/map-blosm-menu.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Configurar o blosm para obter os dados do google
    
<div align="center">
  <img src="images/map-blosm-config.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Apertar o botão “select” que irá abrir no navegador uma aba para selecionar a parte do mapa que se deseja.
    
<div align="center">
  <img src="images/map-select.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Pressionar o botão “Copy” no navegador e em seguida “Paste” na aba do Blosm do blender.
    
<div align="center">
  <img src="images/map-blosm-coord.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Por fim, pressionar o botão “import” e aguardar (a importação pode durar até mais de 5 minutos dependendo do tamanho do mapa). Após finalizar o mapa terá a seguinte aparência.
    
<div align="center">
  <img src="images/map-first-visu.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Para visualizar as texturas basta ativar o viewport com material preview
    
<div align="center">
  <img src="images/map-viewport.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Tendo então a visualização final

<div align="center">
  <img src="images/map-best-visu.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    

# Exportar mapa para o Gazebo

Antes de exportar o mapa para o gazebo, é necessário ajustar a posição do modelo no blender para permitir utilizar as coordenadas de GPS corretamente, para isso utilizaremos um ponto de referência qualquer no mapa e o transformaremos na origem.

## Preparação do mapa

- Ajuste a visão para ter facilidade em selecionar o ponto no mapa, basta utilizar a top-view, pressionando o 7 no numpad do teclado ou pressionando no eixo Z na referência.

<div align="center">
  <img src="images/map-select-axis.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Ative a ferramenta do cursor

<div align="center">
  <img src="images/map-config-cursor.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Em seguida, encontre um ponto que sirva de referência, uma interseção de rua, um objeto ou estrutura pontual; e pressione no ponto, isto irá posicionar o cursor.
    
<div align="center">
  <img src="images/map-select-cursor.png" alt="" style="max-width: 80%; height: auto;" />
</div>

    
- Selecione o objeto no painel do mundo
    
<div align="center">
  <img src="images/map-select-tile.png" alt="" style="max-width: 80%; height: auto;" />
</div>

    
- Mova a origem do objeto para o cursor
    
<div align="center">
  <img src="images/map-origin-cursor.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Pressione “Shift+S” e escolha a opção “Cursor to World Origin”

<div align="center">
  <img src="images/map-cursor-origin.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Pressione “Shift+S” e escolha a opção “Selection to Cursor”.
    
<div align="center">
  <img src="images/map-move-cursor.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Após seguir todos os passos, o ponto desejado estara na posição (0,0,0) do mundo. Isto será importante para utilizar o mapa no gazebo corretamente.

## Exportando mundo

O formato recomendável para exportar mundos compatíveis com o gazebo é o glTF, também é possível com fbx, no entanto, durante testes ele se mostra ineficiente, dado que o gazebo usa plugins a mais para suporta-lo. Logo o tutorial a seguir mostra como exportar o mundo em glTF.

- Selecionar o mundo na aba de objetos
    
<div align="center">
  <img src="images/map-select-tile.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Navegar até File > Export > glTF 2.0

<div align="center">
  <img src="images/map-select-gltf.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Utilizar as seguintes configurações

<div align="center">
  <img src="images/map-config-1.png" alt="" style="max-width: 80%; height: auto;" />
</div>

<div align="center">
  <img src="images/map-config-2.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Escolher o nome do arquivo e onde será salvo, e por fim exportar.
    
<div align="center">
  <img src="images/map-gltf-confirm.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- O mundo é exportado como um arquivo único no formato .glb

<div align="center">
  <img src="images/map-final-file.png" alt="" style="max-width: 80%; height: auto;" />
</div>

## Adicionando o mundo ao Gazebo

Para adicionar o mundo ao gazebo é necessário criar o arquivo de configuração do modelo e posteriormente aplica-lo a um mundo com física e outros objetos. Além disso, é necessário obter as coordenadas GPS do ponto origem do mapa que foi definido passos atrás.

Para obter as coordenadas GPS do ponto de origem do mundo será utilizado o http://earth.google.com/web.

- Ao abrir o site, basta pressionar o botão “Explore Earth” no canto superior direito
    
<div align="center">
  <img src="images/map-earth.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Em seguida deve-se localizar o mesmo ponto de referência utilizado durante a criação do mundo no gazebo.

<div align="center">
  <img src="images/map-earth-ref.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Ao encontrá-lo, basta posicionar o mouse sobre o ponto, pressionar o botão direito do mouse e escolher a opção “Get info”
    
<div align="center">
  <img src="images/map-earth-info.png" alt="" style="max-width: 80%; height: auto;" />
</div>
    
- Um painel surgirá na lateral direita da tela com as informações desejadas, deve-se anotar as coordenadas e a elevação.
    
<div align="center">
  <img src="images/map-earth-values.png" alt="" style="max-width: 80%; height: auto;" />
</div>

- Após isso é necessário converter as coordenadas para a representação decimal: https://latlongdata.com/lat-long-converter (Se atentar a referência da coordenada N/S e E/W)

Por fim, para utilizar o mundo no gazebo é necessário configurar um mundo para usa-lo. Ele terá a seguinte arquitetura

```sh
world/ 
-- mesh/
----- world.glb
-- model.config
-- model.sdf
world.sdf
```

- model.config
    
    > Substituir “WORLD” pelo nome do mundo
    
    <details>
    <summary>model.config</summary>

    ```sh
    <?xml version="1.0"?>
    <model>
      <name>WORLD</name>
      <version>1.0</version>
      <sdf version="1.9">model.sdf</sdf>
    
      <author>
        <name>None</name>
        <email>none</email>
      </author>
    
      <description>
        Environment exported as GLB.
      </description>
    </model>
    
    ```
    </details>
    

- model.sdf
    
    > Substituir “WORLD” pelo nome do mundo
    
    <details>
    <summary>model.sdf</summary>

    ```sh
    <?xml version="1.0" ?>
    <sdf version="1.9">
    
      <model name="WORLD">
        <static>true</static>
    
        <link name="link">
    
          <!-- FAST collision -->
          <collision name="collision">
            <geometry>
              <mesh>
                <uri>model://WORLD/meshes/WORLD.glb</uri>
                <scale>1.0 1.0 1.0</scale>
              </mesh>
            </geometry>
    
            <surface>
              <friction>
                <ode>
                  <mu>1.0</mu>
                  <mu2>1.0</mu2>
                </ode>
              </friction>
            </surface>
          </collision>
    
          <!-- Visual mesh -->
          <visual name="visual">
            <geometry>
              <mesh>
                <uri>model://WORLD/meshes/WORLD.glb</uri>
                <scale>1.0 1.0 1.0</scale>
              </mesh>
            </geometry>
          </visual>
    
        </link>
    
      </model>
    
    </sdf>
    
    ```
    </details>
    
- world.sdf (substituir world pelo nome do mundo)
    
    > Substituir "WORLD" pelo nome do mundo
    
    > Substituir "WORLD_LATITUDE" pelo valor decimal da latitude
    
    > Substituir "WORLD_LONGITUDE" pelo valor decimal da longitude
    
    > substituir "WORLD_ELEVATION" pelo valor decimal da elevação

    <details>
    <summary>world.sdf</summary>
    
    ```sh
    <?xml version="1.0" encoding="UTF-8"?>
    <sdf version="1.9">
      <world name="WORLD">
    
        <!-- Gazebo/PX4 pyhsics, it is overridden when using Ardupilot -->
        <physics type="ode">
          <max_step_size>0.004</max_step_size>
          <real_time_factor>3</real_time_factor>
          <real_time_update_rate>250</real_time_update_rate>
        </physics>
        <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system">
          <engine>
            <filename>libgz-physics-dartsim-plugin</filename>
            <!-- libgz-physics-dartsim-plugin is the default -->
            <!-- The other installed plugins do not allow flight -->
            <!-- libgz-physics-bullet-plugin libgz-physics-bullet-featherstone-plugin libgz-physics-tpe-plugin -->
          </engine>
        </plugin>
        <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
        <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
        <plugin name="gz::sim::systems::Contact" filename="gz-sim-contact-system"/>
        <plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>
        <plugin name="gz::sim::systems::AirPressure" filename="gz-sim-air-pressure-system"/>
        <plugin name="gz::sim::systems::ApplyLinkWrench" filename="gz-sim-apply-link-wrench-system"/>
        <plugin name="gz::sim::systems::NavSat" filename="gz-sim-navsat-system"/>
        <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
          <!-- options: ogre (better performance), ogre2 (better quality) -->
          <render_engine>ogre2</render_engine>
        </plugin>
        <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
          <force_approximation_scaling_factor>1</force_approximation_scaling_factor>
          <horizontal>
            <magnitude> <!-- Randomized strength -->
              <time_for_rise>5</time_for_rise> <!-- 5s to reach full strength -->
              <sin>
                <amplitude_percent>0.1</amplitude_percent> <!-- 10% gusts -->
                <period>30</period>
              </sin>
              <noise type="gaussian">
               <mean>0</mean>
               <stddev>0.0002</stddev>
              </noise>
            </magnitude>
            <direction> <!-- Randomized direction -->
              <time_for_rise>10</time_for_rise> <!-- 10s to adjust to new bearing -->
              <sin>
                <amplitude>5</amplitude> <!-- 5deg oscillations in bearing -->
                <period>20</period>
              </sin>
              <noise type="gaussian">
               <mean>0</mean>
               <stddev>0.03</stddev>
              </noise>
            </direction>
          </horizontal>
          <vertical> <!-- Updrafts and downdrafts -->
            <noise type="gaussian">
             <mean>0</mean>
             <stddev>0.03</stddev>
            </noise>
          </vertical>
        </plugin>
    
        <gui fullscreen="false">
          <!-- 3D scene -->
          <plugin filename="MinimalScene" name="3D View">
            <gz-gui>
              <title>3D View</title>
              <property type="bool" key="showTitleBar">false</property>
              <property type="string" key="state">docked</property>
            </gz-gui>
            <engine>ogre2</engine>
            <scene>scene</scene>
            <ambient_light>0.4 0.4 0.4</ambient_light>
            <background_color>0.8 0.8 0.8</background_color>
            <camera_pose>-6 0 6 0 0.5 0</camera_pose>
            <camera_clip>
              <near>0.25</near>
              <far>2000</far>
            </camera_clip>
          </plugin>
          <!-- Plugins that add functionality to the scene -->
          <plugin filename="EntityContextMenuPlugin" name="Entity context menu">
            <gz-gui>
              <property key="state" type="string">floating</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="showTitleBar" type="bool">false</property>
            </gz-gui>
          </plugin>
          <plugin filename="GzSceneManager" name="Scene Manager">
            <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
            </gz-gui>
          </plugin>
          <plugin filename="InteractiveViewControl" name="Interactive view control">
            <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
            </gz-gui>
          </plugin>
          <plugin filename="CameraTracking" name="Camera Tracking">
            <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
            </gz-gui>
          </plugin>
          <plugin filename="MarkerManager" name="Marker manager">
            <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
            </gz-gui>
          </plugin>
          <plugin filename="SelectEntities" name="Select Entities">
            <gz-gui>
              <anchors target="Select entities">
                <line own="right" target="right"/>
                <line own="top" target="top"/>
              </anchors>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
            </gz-gui>
          </plugin>
          <plugin filename="VisualizationCapabilities" name="Visualization Capabilities">
            <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
            </gz-gui>
          </plugin>
          <plugin filename="Spawn" name="Spawn Entities">
            <gz-gui>
              <anchors target="Select entities">
                <line own="right" target="right"/>
                <line own="top" target="top"/>
              </anchors>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
            </gz-gui>
          </plugin>
          <plugin name="World control" filename="WorldControl">
            <gz-gui>
              <title>World control</title>
              <property type="bool" key="showTitleBar">0</property>
              <property type="bool" key="resizable">0</property>
              <property type="double" key="height">72</property>
              <property type="double" key="width">121</property>
              <property type="double" key="z">1</property>
              <property type="string" key="state">floating</property>
              <anchors target="3D View">
                <line own="left" target="left"/>
                <line own="bottom" target="bottom"/>
              </anchors>
            </gz-gui>
            <play_pause>1</play_pause>
            <step>1</step>
            <start_paused>1</start_paused>
          </plugin>
          <plugin name="World stats" filename="WorldStats">
            <gz-gui>
              <title>World stats</title>
              <property type="bool" key="showTitleBar">0</property>
              <property type="bool" key="resizable">0</property>
              <property type="double" key="height">110</property>
              <property type="double" key="width">290</property>
              <property type="double" key="z">1</property>
              <property type="string" key="state">floating</property>
              <anchors target="3D View">
                <line own="right" target="right"/>
                <line own="bottom" target="bottom"/>
              </anchors>
            </gz-gui>
            <sim_time>1</sim_time>
            <real_time>1</real_time>
            <real_time_factor>1</real_time_factor>
            <iterations>1</iterations>
          </plugin>
          <plugin name="Entity tree" filename="EntityTree">
            <gz-gui>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">true</property>
              <property key="width" type="double">150</property>
              <property key="height" type="double">4096</property>
            </gz-gui>
          </plugin>
        </gui>
    
        <gravity>0 0 -9.8</gravity>
        <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
        <atmosphere type="adiabatic"/>
        <scene>
          <grid>false</grid>
          <ambient>0.4 0.4 0.4 1</ambient>
          <background>0.7 0.7 0.7 1</background>
          <shadows>false</shadows>
          <sky></sky>
        </scene>
    
        <spherical_coordinates>
          <surface_model>EARTH_WGS84</surface_model>
          <world_frame_orientation>ENU</world_frame_orientation>
          <latitude_deg>WORLD_LATITUDE</latitude_deg> 
          <longitude_deg>WORLD_LONGITUDE</longitude_deg>
          <elevation>WORLD_ELEVATION</elevation>
        </spherical_coordinates>
    
        <wind>
          <linear_velocity>0.0 0.0 0.0</linear_velocity> <!-- positive X blows from West, positive Y blows from South  -->
        </wind>
    
        <model name="objects">
          <static>true</static>
          <pose>0 0 0 0 0 0</pose>
    
          <link name="lighting_link">
            <pose>0 0 0 0 0 0</pose>
            <light name="sun" type="directional">
              <pose>0 0 500 0 -0 0</pose>
              <cast_shadows>false</cast_shadows>
              <intensity>1</intensity>
              <direction>0.001 0.625 -0.78</direction>
              <diffuse>0.904 0.904 0.904 1</diffuse>
              <specular>0.271 0.271 0.271 1</specular>
              <attenuation>
                <range>2000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
              </attenuation>
              <spot>
                <inner_angle>0</inner_angle>
                <outer_angle>0</outer_angle>
                <falloff>0</falloff>
              </spot>
            </light>
          </link>
    
          <model name="axes">
            <static>1</static>
            <link name="link">
              <visual name="r">
                <cast_shadows>0</cast_shadows>
                <pose>5 0 0.1 0 0 0</pose>
                <geometry>
                  <box>
                    <size>10 0.01 0.01</size>
                  </box>
                </geometry>
                <material>
                  <ambient>1 0 0 0.8</ambient>
                  <diffuse>1 0 0 0.8</diffuse>
                  <emissive>1 0 0 0.8</emissive>
                  <specular>0.5 0.5 0.5 0.8</specular>
                </material>
              </visual>
              <visual name="g">
                <cast_shadows>0</cast_shadows>
                <pose>0 5 0.1 0 0 0</pose>
                <geometry>
                  <box>
                    <size>0.01 10 0.01</size>
                  </box>
                </geometry>
                <material>
                  <ambient>0 1 0 0.8</ambient>
                  <diffuse>0 1 0 0.8</diffuse>
                  <emissive>0 1 0 0.8</emissive>
                  <specular>0.5 0.5 0.5 0.8</specular>
                </material>
              </visual>
              <visual name="b">
                <cast_shadows>0</cast_shadows>
                <pose>0 0 5.1 0 0 0</pose>
                <geometry>
                  <box>
                    <size>0.01 0.01 10</size>
                  </box>
                </geometry>
                <material>
                  <ambient>0 0 1 0.8</ambient>
                  <diffuse>0 0 1 0.8</diffuse>
                  <emissive>0 0 1 0.8</emissive>
                  <specular>0.5 0.5 0.5 0.8</specular>
                </material>
              </visual>
              <sensor name="navsat_sensor" type="navsat">
                <always_on>1</always_on>
                <update_rate>1</update_rate>
              </sensor>
            </link>
          </model>
    
          <model name="ground_plane">
            <pose>0 0 0 0 0 0</pose> <static>true</static>
            <link name="link">
              <pose>0 0 0 0 -0 0</pose>
            </link>
            <pose degrees="true">0 0 0 0 0 0</pose>
            <self_collide>false</self_collide>
          </model>
    
        </model>
    
        <include>
          <uri>model://WORLD</uri>
          <pose>0 0 0 0 0 0</pose>
        </include>
    
        <include>
          <uri>https://fuel.gazebosim.org/1.0/saiaravind19/models/helipad</uri>
          <pose> 60.12 183.40 2.3 0 0 0.26 </pose>
        </include>
    
      </world>
    </sdf>
    
    ```
    </details>
    

- Após criar todos os arquivos na estrutura mostrada acima e ter adicionado o arquivo .glb do mundo exportado do blender, basta copiar os arquivos na pasta “aerial-autonomy-stack/simulation/simulation_resources/simulation_worlds”
- Para executar, basta utilizar o nome do mundo na inicialização da simulação com a flag WORLD, ex.:

```sh
AUTOPILOT=ardupilot NUM_QUADS=1 WORLD=big_esefex RTF=1.0 CENTRALIZED=true ./sim_run.sh
```