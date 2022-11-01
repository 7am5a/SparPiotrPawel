#ifndef MENU
#define MENU

/**
 * @brief Definition of menu components (*name, *next, *prev, *child, *parent, (*menu_function))
 *  Can be one of 3 parameters: menu_t struct or NULL or function pointer to callback
 * 
 */
typedef struct  menu_struct menu_t;

struct menu_struct {
    const char * name;
    menu_t * next;
    menu_t * prev;
    menu_t * child;
    menu_t * parent;
    void (*menu_function)(void);
};

menu_t menu1;
    menu_t sub_menu1_1;
    menu_t sub_menu1_2;
        menu_t sub_menu1_2_1;
    menu_t sub_menu1_3;
menu_t menu2;
    menu_t sub_menu2_1;
menu_t menu3;
    menu_t sub_menu3_1;

/**
 * @brief Enable shift to next menu option.
 * 
 */
void menu_next();

/**
 * @brief Enable shift to previous menu option.
 * 
 */
void menu_prev();

/**
 * @brief Take index from choosen menu position.
 * 
 * @param index Menu position
 * @return Return value of index
 */
uint8_t menu_get_index(menu_t *index);

/**
 * @brief Take menu level from choosen position
 * 
 * @param level Menu position
 * @return Return value of level
 */
uint8_t menu_get_level(menu_t *level);

/**
 * @brief Enable enter from higher to lower level of menu/submenu.
 *        There has to be one case less than max level of menu.
 */
void menu_enter();

/**
 * @brief Enable to go back from lower to higher level of menu/submenu.
 * 
 */
void menu_back();

/**
 * @brief Refresh displayed menu afrer any operation
 * 
 */
void menu_refresh();

/**
 * @brief Main menu function
 * 
 */
void menu_task();

/**
 * @brief Create menu task
 * 
 */
void set_menu_task();

#endif