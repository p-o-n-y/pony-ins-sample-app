/*
	Пример приложения на архитектуре pony [https://github.com/p-o-n-y/pony], 
	включая ядро и частные алгоритмы инерциальной навигации.

	А. Козлов
	Лаборатория управления и навигации МГУ
	2020
*/

// заголовочные файлы стандартных библиотек C89
#include <stdio.h>
#include <stdlib.h>

// заголовочные файлы библиотек лаборатории
#include "../pony/pony.h"
#include "../pony/ins/pony_ins_gravity.h"
#include "../pony/ins/pony_ins_alignment.h"
#include "../pony/ins/pony_ins_attitude.h"
#include "../pony/ins/pony_ins_motion.h"

// проверка версии ядра
#define PONY_INS_SAMPLE_APP_PONY_BUS_VERSION_REQUIRED 8
#if PONY_BUS_VERSION < PONY_INS_SAMPLE_APP_PONY_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the newest one"
#endif

#define PONY_INS_SAMPLE_APP_BUFFER_SIZE 1024

// частные алгоритмы приложения
void pony_ins_sample_app_step_sync     (void);
void pony_ins_sample_app_read_input    (void);
void pony_ins_sample_app_write_output  (void);
void pony_ins_sample_app_print_progress(void);

void main(void)
{
	const char pony_configuration[] = 
		"{imu: lat = +55.0000302831, lon = +37.0000369619, alt = +200, alignment = 30, freq = 400, vertical_damping_stdev = 0}, sensors_in = wavesim.txt out = ins.nav";

	printf("pony-ins-sample-app has started\n----\n");

	// добавление частных алгоритмов
	if (   !pony->add_plugin(pony_ins_sample_app_step_sync     ) // ожидание метки времени шага навигационного решения
		|| !pony->add_plugin(pony_ins_sample_app_read_input    ) // получение показаний датчиков
		|| !pony->add_plugin(pony_ins_gravity_normal           ) // модель поля силы тяжести
		|| !pony->add_plugin(pony_ins_alignment_static         ) // начальная выставка
		|| !pony->add_plugin(pony_ins_attitude_rodrigues       ) // ориентация
		|| !pony->add_plugin(pony_ins_motion_euler             ) // положение и скорость
		|| !pony->add_plugin(pony_ins_motion_vertical_damping  ) // демпфирование в вертикальном канале
		|| !pony->add_plugin(pony_ins_sample_app_write_output  ) // запись навигационного решения
		|| !pony->add_plugin(pony_ins_sample_app_print_progress) // вывод на экран
		) {
		printf("ERROR on initialization\n"); // ошибка добавления частных алгоритмов
		return;									
	}

	// инициализация ядра
	if (pony->init((char*)pony_configuration))
		while(pony->step()); // основной цикл

	// ошибка инициализации
	else {
		printf("ERROR on initialization\n");
		return;
	}

	printf("\n----\npony-ins-sample-app has terminated\n");
}

/*
	шаг времени инерциального решения
	использует:
		не использует данные шины	
	изменяет:
		imu->t
		imu->w_valid
		imu->f_valid
	параметры:
		{imu: freq} — частота работы навигационного алгоритма, Гц
			тип: число
			диапазон: 50-3200
			значение по умолчанию: 100
			пример: {imu: freq = 400}
*/
void pony_ins_sample_app_step_sync(void) 
{
	const char   freq_token[] = "freq";     // имя параметра в строке конфигурации, содержащего частоту
	const double freq_range[] = {50, 3200}; // диапазон допустимых частот
	const double freq_default = 100;        // частота по умолчанию

	static double        dt = -1;           // шаг по времени
	static unsigned long  i =  0;           // номер шага

	char *freq_ptr;                         // указатель на значение частоты в строке конфигурации

	// проверка инициализации инерциальной подсистемы на шине
	if (pony->imu == NULL)
		return;

	// инициализация
	if (pony->mode == 0) {
		// начальные значения
		pony->t = 0;
		i = 0;
		// поиск значения частоты в конфигурации
		freq_ptr = pony_locate_token(freq_token, pony->imu->cfg, pony->imu->cfglength, '=');
		if (freq_ptr != NULL)
			dt = atof(freq_ptr);	
		// если значение не найдено в конфигурации, или значение вне заданных пределов, установка по умолчанию
		if (freq_ptr == NULL || dt < freq_range[0] || freq_range[1] < dt)
			dt = freq_default;
		// вычисление шага по времени
		dt = 1 / dt;
	}

	// завершение работы
	else if (pony->mode < 0) {
		// ничего не делать
	}

	// шаг основного цикла	
	else {
		// сброс достоверности инерциальных датчиков
		pony->imu->w_valid = 0;
		pony->imu->f_valid = 0;
		// шаг по времени
		i++;
		pony->imu->t = i*dt;
	}
}

/*	
	чтение показаний инерциальных датчиков из файла
	использует:
		не использует данные шины	
	изменяет:
		pony->imu.w
		pony->imu.w_valid
		pony->imu.f
		pony->imu.f_valid
	параметры:
		sensors_in — имя входного файла
			тип: строка
			пример: sensors_in = imu.txt
			без пробелов в имени
			с пробелом в конце
*/
void pony_ins_sample_app_read_input(void)
{
	const char input_file_token[] = "sensors_in";		  // имя параметра конфигурации с входным файлом
	const int  n0 = 6;                                    // требуемое количество параметров в строке входного файла
														  
	static FILE *fp = NULL;                               // указатель на файл
	static char  buffer[PONY_INS_SAMPLE_APP_BUFFER_SIZE]; // строковый буфер

	char *cfg_ptr; // указатель на параметр в строке конфигурации
	int   n;       // количество параметров в строке


	// проверка инерциальной подсистемы на шине
	if (pony->imu == NULL)
		return;

	// инициализация
	if (pony->mode == 0) {
		// поиск имени входного файла в конфигурации
		cfg_ptr = pony_locate_token(input_file_token, pony->cfg_settings, pony->settings_length, '=');
		if (cfg_ptr != NULL)
			sscanf(cfg_ptr, "%s", buffer);
		// открытие файла
		fp = fopen(buffer, "r");
		if (fp == NULL) {
			printf("ERROR opening input file '%s'\n", buffer);
			pony->mode = -1;
			return;
		}
		// считывание заголовка
		fgets(buffer, PONY_INS_SAMPLE_APP_BUFFER_SIZE, fp);
		// обеспечить завершение строки нулевым символом
		buffer[PONY_INS_SAMPLE_APP_BUFFER_SIZE-1] = '\0';
	}

	// завершение работы
	else if (pony->mode < 0) {
		if (fp != NULL)
			fclose(fp); // закрытие файла, если он был открыт
		return;
	}

	// основной цикл
	else {
		// сброс флагов достоверности показаний датчиков
		pony->imu->w_valid = 0;
		pony->imu->f_valid = 0;
		// чтение строки из файла
		if (fgets(buffer, PONY_INS_SAMPLE_APP_BUFFER_SIZE, fp) == NULL) {
			pony->mode = -1; // завершение работы, если не удалось прочитать строку
			return;
		}
		// парсинг строки
		n = sscanf(buffer, "%lg %lg %lg %lg %lg %lg",
			&(pony->imu->w[0]), &(pony->imu->w[1]), &(pony->imu->w[2]),
			&(pony->imu->f[0]), &(pony->imu->f[1]), &(pony->imu->f[2]));
		if (n < n0) // недостаточно параметров в строке
			return;
		// перевод ДУС из градусов в радианы
		for (n = 0; n < 3; n++)
			pony->imu->w[n] /= pony->imu_const.rad2deg;
		// установка флагов достоверности
		pony->imu->w_valid = 1;
		pony->imu->f_valid = 1;
	}
}

/*
	запись навигационного решения в файл
	использует:
		pony->imu->t
		pony->imu.sol
	изменяет:
	параметры:
		out — имя выходного файла
			тип: строка
			пример: {imu: out = ins.txt }
			без пробелов в имени
			с пробелом в конце
*/
void pony_ins_sample_app_write_output(void)
{
	const char nav_file_token[]	= "out"; // имя параметра в конфигурации с выходным файлом
	const int  fmt[]            =        // количество выводимых символов всего и после запятой, для каждого параметра по порядку
		{11,5,   13,8,  12,8,  9,3,   10,4, 10,4, 10,4, 13,8,   12,8,    13,8};
	//   time    lon    lat    alt    Ve    Vn    Vu    roll    pitch    yaw

	static FILE *fp = NULL;                               // указатель на файл
	static char  buffer[PONY_INS_SAMPLE_APP_BUFFER_SIZE]; // строковый буфер

	char *cfg_ptr; // указатель на параметр в строке конфигурации
	int   i, j;    // индексы

	// проверка инерциальной подсистемы на шине
	if (pony->imu == NULL)
		return;

	// инициализация частного алгоритма
	if (pony->mode == 0) {
		// поиск имени выходного файла в конфигурации
		cfg_ptr = pony_locate_token(nav_file_token, pony->cfg, pony->cfglength, '=');
		if (cfg_ptr != NULL)
			sscanf(cfg_ptr, "%s", buffer);
		// открытие файла
		fp = fopen(buffer, "w");
		if (fp == NULL) {
			printf("ERROR opening output file '%s'\n", buffer);
			pony->mode = -1;
			return;
		}		
		// строка заголовка
		fprintf(fp, "%%   time[s]||   lon[deg]  |  lat[deg]  |   alt[m]||   Ve[m/s]|   Vn[m/s]|   Vu[m/s]||    roll[deg]|  pitch[deg]| heading[deg]|");
	}

	// завершение работы
	else if (pony->mode < 0) {
		if (fp != NULL)
			fclose(fp); // закрытие файла, если он был открыт
		return;
	}

	// операции на каждом шаге
	else {
		// вывод навигационного решения в файл
		j = 0; 
		//   time    lon    lat    alt    Ve    Vn    Vu    roll    pitch    yaw
		                                         fprintf(fp, "\n% *.*f", fmt[j],fmt[j+1], pony->imu->t                                 ), j += 2;
		fprintf(fp," "); for (i = 0; i < 2; i++) fprintf(fp, " % *.*f",  fmt[j],fmt[j+1], pony->imu->sol.llh[i]*pony->imu_const.rad2deg), j += 2;
		                                         fprintf(fp, " % *.*f",  fmt[j],fmt[j+1], pony->imu->sol.llh[2]                        ), j += 2;
		fprintf(fp," "); for (i = 0; i < 3; i++) fprintf(fp, " % *.*f",  fmt[j],fmt[j+1], pony->imu->sol.v  [i]                        ), j += 2;
		fprintf(fp," "); for (i = 0; i < 3; i++) fprintf(fp, " % *.*f",  fmt[j],fmt[j+1], pony->imu->sol.rpy[i]*pony->imu_const.rad2deg), j += 2;
	}
}

/*
	вывод текущего времени на экран
	использует:
		pony->imu->t
	изменяет:
		не изменяет данные шины
	параметры:
		нет использует параметры
*/
void pony_ins_sample_app_print_progress(void)
{	
	const char bkspc[] = "\b\b\b\b\b"; // возврат курсора
	const int 
		decimals = sizeof(bkspc)-1,    // количество выводимых десятичных знаков целой части
		interval = 1024;               // интервал вывода, в шагах
	static long counter;               // счётчик

	// проверка инерциальной подсистемы на шине
	if (pony->imu == NULL)
		return;

	// инициализация частного алгоритма
	if (pony->mode == 0) {
		printf("seconds into navigation: % *.0f", decimals, pony->imu->t);
		counter = 0; // сброс счётчика
	}

	// завершение работы
	else if (pony->mode < 0)
		printf("%s% *.0f", bkspc, decimals, pony->imu->t); // печать конечного времени

	// операции на каждом шаге
	else {
		// печать на экран
		if (counter%interval == 0)
			printf("%s% *.0f", bkspc, decimals, pony->imu->t);
		counter++;
	}
}
