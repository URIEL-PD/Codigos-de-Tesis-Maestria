# Códigos y datos experimentales de tesis de maestría

Repositorio complementario a la tesis:

> **Localización de una fuente RF mediante mediciones RSSI y estimación del ángulo de llegada utilizando un dron cuadricóptero autopiloteado**

**Autor:** Kevin Uriel Pérez Delgado  
**Institución:** Centro de Investigación y de Estudios Avanzados del Instituto Politécnico Nacional (CINVESTAV-IPN)  
**Programa:** Maestría en Ciencias en Sistemas Autónomos de Navegación Aérea y Submarina  
**Asesores:** Dr. Aldo Gustavo Orozco Lugo y Dr. Moisés Bonilla Estrada  

## Descripción

Este repositorio reúne los códigos desarrollados, datos experimentales, figuras y registros de vuelo utilizados durante el desarrollo de la tesis. Su propósito es complementar el contenido del documento escrito y facilitar la consulta de los programas y resultados experimentales descritos en los capítulos correspondientes.

El material se encuentra organizado de acuerdo con la estructura de la tesis. Los Capítulos 1 y 2 son principalmente introductorios y teóricos, por lo que el repositorio comienza en el Capítulo 3, donde inicia el desarrollo computacional del sistema.

A partir del Capítulo 5, además de los códigos desarrollados, se incluyen los datos experimentales y registros utilizados para obtener y analizar los resultados presentados en la tesis.

## Estructura del repositorio

| Capítulo | Contenido principal |
|---|---|
| `Capítulo 3` | Programas desarrollados para establecer la comunicación entre dos computadoras mediante módulos MRAPC, incluyendo registro en la red, construcción y procesamiento de tramas, cálculo de CRC y transmisión de mensajes entre una PC y una Raspberry Pi 5. |
| `Capítulo 4` | Programas para la solicitud y adquisición periódica de mediciones RSSI, conversión de las lecturas a dBm y aplicación del filtro digital IIR de primer orden utilizado para suavizar la señal. |
| `Capítulo 5` | Códigos y datos experimentales asociados con el sistema automatizado de medición de patrones de radiación en tierra, el control de la estación móvil direccional, la adquisición de RSSI, la construcción de patrones polares y la estimación del ángulo de llegada (AoA). |
| `Capítulo 6` | Códigos y datos experimentales relacionados con la integración de PX4 y MAVSDK para navegación autónoma en modo *Offboard*, incluyendo los registros de la misión utilizada para validar el seguimiento de referencias de posición, altura, velocidad y orientación del cuadricóptero. |
| `Capítulo 7` | Códigos y datos experimentales correspondientes a los barridos angulares continuos y discretos realizados durante el vuelo, incluyendo los programas de la Raspberry Pi y de la estación terrena, patrones RSSI y registros asociados con la estimación del AoA. También se incluyen los registros de PX4. |
| `Capítulo 8` | Códigos y datos experimentales correspondientes a la misión final de seguimiento progresivo hacia la fuente de RF, incluyendo los programas de navegación y estación terrena, los barridos realizados, las figuras utilizadas en el análisis y el registro PX4 de la prueba final. |
| `Manuales` | Documentación técnica utilizada como apoyo durante el desarrollo, incluyendo el manual de los nodos RF/MRAPC empleados en el sistema experimental. |

## Tipos de archivos incluidos

Dependiendo del capítulo, el repositorio contiene:

- **`.py`**: programas desarrollados en Python para comunicación, adquisición de RSSI, procesamiento de datos, interfaces gráficas y control del cuadricóptero.
- **`.csv`**: datos experimentales obtenidos durante los barridos angulares y las misiones de vuelo.
- **`.ulg`**: registros de vuelo generados por PX4.
- **`.png` y `.pdf`**: figuras, patrones de radiación, mapas y gráficas utilizadas para documentar los resultados.
- **`.pdf`**: manuales y documentación técnica de los dispositivos utilizados.

## Capítulo 8: prueba final de seguimiento hacia la fuente de RF

La prueba final integra la estimación del AoA con la navegación autónoma del cuadricóptero. La misión comienza con un barrido angular discreto completo y posteriormente realiza desplazamientos de **10 m** intercalados con semibarridos discretos para actualizar la dirección de avance.

Las direcciones estimadas durante la misión fueron:

| Etapa | AoA estimado |
|---|---:|
| Barrido completo | 315° |
| Primer semibarrido | 300° |
| Segundo semibarrido | 305° |

Los principales datos experimentales de esta prueba se encuentran en:

```text
Capítulo 8/
├── datos_experimentales/
│   ├── barrido_completo.csv
│   ├── semi_barrido1.csv
│   ├── semi_barrido2.csv
│   └── mision_general.csv
├── figuras/
│   ├── mapa_distancia_dron_fuente.pdf
│   ├── barrido_completo.png
│   ├── semi_barrido1.png
│   ├── semi_barrido2.png
│   ├── RSSI_mision_seguimiento.pdf
│   ├── mapa_seguimiento_dron_fuente.pdf
│   └── mapa_ubicacion_estimada_de_la_fuente.pdf
└── log_px4/
    └── log_25_2026-8-20-10-40-34.ulg
```

El archivo `log_25_2026-8-20-10-40-34.ulg` corresponde a la misión final documentada en la tesis. En este registro se observa la secuencia de orientación utilizada durante el seguimiento: barrido completo, avance hacia **315°**, primer semibarrido, avance hacia **300°**, segundo semibarrido y avance final hacia **305°**.

La misión terminó al cumplirse el criterio de proximidad basado en RSSI y produjo una estimación de la ubicación de la fuente con una separación horizontal aproximada de **2.92 m** respecto a la posición de referencia utilizada durante la prueba.

## Software y herramientas principales

El desarrollo experimental emplea principalmente:

- **Python 3**
- **PySerial** para comunicación serie.
- **Matplotlib** para visualización de RSSI y patrones de radiación.
- **Tkinter** para las interfaces de la estación terrena.
- **MAVSDK** para el envío de comandos y referencias de navegación desde la Raspberry Pi 5.
- **PX4** como firmware del autopiloto.
- **QGroundControl** para configuración, supervisión y telemetría del cuadricóptero.

Las dependencias específicas pueden variar entre capítulos y se indican dentro de los programas correspondientes.

## Consideraciones para ejecutar los programas

Los programas fueron desarrollados para la configuración experimental utilizada en la tesis. Antes de ejecutarlos en otro sistema se deben revisar, entre otros parámetros, los puertos serie, velocidades de comunicación, direcciones de los módulos MRAPC, parámetros de PX4 y conexiones físicas empleadas.

En particular, los programas de los Capítulos 6, 7 y 8 controlan un cuadricóptero real mediante PX4/MAVSDK. Su ejecución debe realizarse únicamente después de verificar la configuración del vehículo, los mecanismos de seguridad y la posibilidad de intervención manual mediante radio control.

## Datos experimentales y reproducibilidad

Los archivos CSV contienen los datos utilizados para construir las tablas y figuras presentadas en la tesis. Los archivos ULog conservan los registros originales generados por PX4 durante las pruebas de vuelo seleccionadas.

El procedimiento experimental, los parámetros utilizados y el análisis de los resultados se describen con detalle en la tesis. Por esta razón, se recomienda consultar el capítulo correspondiente junto con los archivos de este repositorio.

## Trabajo relacionado

Parte del sistema experimental desarrollado para la medición automatizada de patrones RSSI y la estimación del AoA fue presentada en el trabajo:

> **Automated RSSI-Based Radiation Pattern Measurement for Angle-of-Arrival Estimation in RF Source Localization**

El desarrollo posterior de la tesis extendió este sistema a pruebas realizadas directamente durante el vuelo del cuadricóptero y a una estrategia de seguimiento progresivo hacia la fuente de RF.

## Uso académico

Este repositorio se publica como material complementario de una tesis de maestría y tiene como finalidad facilitar la consulta y reproducción de los procedimientos desarrollados. Si se utilizan los códigos, datos experimentales o resultados aquí presentados en otro trabajo académico, se recomienda citar la tesis y, cuando corresponda, la publicación asociada.

---

**CINVESTAV-IPN, Ciudad de México, México — 2026**
